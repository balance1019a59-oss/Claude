"""Realise a :class:`core.plan.RigPlan` in the current Blender scene."""

import math
from collections import defaultdict

import bpy
from mathutils import Matrix, Vector

from .core.plan import make_plan
from .core.schema import validate

COLLECTION_VISIBILITY = {"CTRL": True, "DEF": True, "JNT": False, "MCH": False}


class BuildResult:
    def __init__(self, rig, plan, mesh_objects, warnings):
        self.rig, self.plan, self.mesh_objects, self.warnings = rig, plan, mesh_objects, warnings


def import_rig(context, data, scale=0.01, primary_axis="AUTO", name=None, meshes=True,
               constraints=True, drivers=True, widgets=True):
    """Validate ``data``, plan it and build it.  Returns :class:`BuildResult`."""
    warnings = validate(data)
    plan = make_plan(data, scale=scale, primary_axis=primary_axis, name=name)
    warnings = warnings + plan.warnings
    builder = _Builder(context, plan, warnings)
    builder.build(meshes=meshes, constraints=constraints, drivers=drivers, widgets=widgets)
    return BuildResult(builder.rig, plan, builder.mesh_objects, warnings)


class _Builder:
    def __init__(self, context, plan, warnings):
        self.context, self.plan, self.warnings = context, plan, warnings
        self.mesh_objects = {}

    def warn(self, msg):
        self.warnings.append(msg)

    # ------------------------------------------------------------------
    def build(self, meshes, constraints, drivers, widgets):
        plan = self.plan
        self.collection = bpy.data.collections.new(plan.name)
        self.context.scene.collection.children.link(self.collection)

        arm = bpy.data.armatures.new(plan.name)
        rig = bpy.data.objects.new(plan.name, arm)
        self.collection.objects.link(rig)
        arm.display_type = "STICK"
        rig.show_in_front = True
        self.rig = rig

        self._edit_bones()
        self._bone_collections()
        self._pose_bones(widgets)
        for prop in plan.object_props:
            self._set_prop(rig, prop)
        if constraints:
            self._constraints()
        if meshes:
            self._meshes()
        if drivers:
            self._drivers()
        if constraints:
            self._verify_pole_angles()
        self._log()
        self.context.view_layer.update()

    def _edit_bones(self):
        rig, ctx = self.rig, self.context
        for obj in ctx.view_layer.objects.selected:
            obj.select_set(False)
        ctx.view_layer.objects.active = rig
        rig.select_set(True)
        bpy.ops.object.mode_set(mode="EDIT")
        ebs = rig.data.edit_bones
        for bp in self.plan.bones:
            eb = ebs.new(bp.name)
            eb.head = (0.0, 0.0, 0.0)
            eb.tail = (0.0, max(bp.length, 1e-4), 0.0)
            eb.matrix = Matrix(bp.matrix)
            eb.use_deform = bp.deform
            eb.inherit_scale = bp.inherit_scale
            eb.use_connect = False
        for bp in self.plan.bones:
            if bp.parent:
                ebs[bp.name].parent = ebs[bp.parent]
        bpy.ops.object.mode_set(mode="OBJECT")

    def _bone_collections(self):
        arm = self.rig.data
        colls = {}
        for bp in self.plan.bones:
            if bp.collection not in colls:
                colls[bp.collection] = arm.collections.new(bp.collection)
                colls[bp.collection].is_visible = COLLECTION_VISIBILITY.get(bp.collection, True)
            colls[bp.collection].assign(arm.bones[bp.name])
            arm.bones[bp.name].hide = bp.hide

    def _widget_collection(self):
        coll = bpy.data.collections.new(self.plan.name + "_widgets")
        self.collection.children.link(coll)
        coll.hide_render = True
        layer = self.context.view_layer.layer_collection.children[self.collection.name].children[coll.name]
        layer.exclude = True
        return coll

    def _pose_bones(self, widgets):
        pose = self.rig.pose
        wgt_coll = None
        for bp in self.plan.bones:
            pb = pose.bones[bp.name]
            pb.rotation_mode = bp.rotation_mode
            pb.lock_location = bp.lock_location
            pb.lock_rotation = bp.lock_rotation
            pb.lock_scale = bp.lock_scale
            for prop in bp.props:
                self._set_prop(pb, prop)
            if bp.color:
                c = Vector(bp.color[:3])
                pb.color.palette = "CUSTOM"
                pb.color.custom.normal = c
                pb.color.custom.select = c.lerp(Vector((1.0, 1.0, 1.0)), 0.4)
                pb.color.custom.active = c.lerp(Vector((1.0, 1.0, 1.0)), 0.7)
            if widgets and bp.shape:
                if wgt_coll is None:
                    wgt_coll = self._widget_collection()
                verts, edges = bp.shape
                mesh = bpy.data.meshes.new("WGT-" + bp.name)
                mesh.from_pydata(verts, edges, [])
                obj = bpy.data.objects.new(mesh.name, mesh)
                wgt_coll.objects.link(obj)
                pb.custom_shape = obj
                pb.use_custom_shape_bone_size = False

    def _set_prop(self, owner, prop):
        owner[prop.name] = prop.value
        if isinstance(prop.value, bool):
            return
        kwargs = {"description": prop.description}
        if prop.min is not None:
            kwargs["min"] = kwargs["soft_min"] = prop.min
        if prop.max is not None:
            kwargs["max"] = kwargs["soft_max"] = prop.max
        if prop.subtype:
            kwargs["subtype"] = prop.subtype
        try:
            owner.id_properties_ui(prop.name).update(**kwargs)
        except (TypeError, ValueError) as exc:
            self.warn("property %s: %s" % (prop.name, exc))
        try:
            owner.property_overridable_library_set('["%s"]' % prop.name, True)
        except (AttributeError, TypeError, ValueError):
            pass

    # ------------------------------------------------------------------
    def _constraints(self):
        rig = self.rig
        for cp in self.plan.constraints:
            pb = rig.pose.bones[cp.owner]
            con = pb.constraints.new(cp.type)
            con.name = cp.name
            if cp.type == "ARMATURE":
                for t in cp.targets:
                    tgt = con.targets.new()
                    tgt.target = rig
                    tgt.subtarget = t["subtarget"]
                    tgt.weight = t["weight"]
            elif cp.targets:
                t = cp.targets[0]
                con.target = rig
                con.subtarget = t["subtarget"]
                if t.get("pole_subtarget"):
                    con.pole_target = rig
                    con.pole_subtarget = t["pole_subtarget"]
            for key, value in cp.settings.items():
                try:
                    setattr(con, key, value)
                except (AttributeError, TypeError, ValueError) as exc:
                    self.warn("constraint %s on %s: cannot set %s (%s)" % (cp.name, cp.owner, key, exc))

    def _chain_rest_error(self, pb, count):
        self.rig.update_tag()
        self.context.view_layer.update()
        worst = 0.0
        while pb is not None and count > 0:
            worst = max(worst, max(abs(a - b) for ra, rb in zip(pb.matrix, pb.bone.matrix_local)
                                   for a, b in zip(ra, rb)))
            pb, count = pb.parent, count - 1
        return worst

    def _verify_pole_angles(self):
        """The planner's pole angle assumes bones point down the chain; check it
        against the rest pose and fall back to the other sign conventions."""
        tol = 1e-4 * max(1.0, max((b.length for b in self.rig.data.bones), default=1.0))
        for cp in self.plan.constraints:
            if cp.type != "IK" or "pole_angle" not in cp.settings:
                continue
            pb = self.rig.pose.bones[cp.owner]
            con = pb.constraints[cp.name]
            if con.influence == 0.0:
                continue
            base = con.pole_angle
            best = (self._chain_rest_error(pb, con.chain_count), base)
            for candidate in (base + math.pi, -base, math.pi - base):
                if best[0] <= tol:
                    break
                con.pole_angle = math.atan2(math.sin(candidate), math.cos(candidate))
                err = self._chain_rest_error(pb, con.chain_count)
                if err < best[0]:
                    best = (err, con.pole_angle)
            con.pole_angle = best[1]
            if best[0] > tol:
                self.warn("IK %s: chain moves by %.4g at rest, check the pole target" % (cp.name, best[0]))

    # ------------------------------------------------------------------
    def _meshes(self):
        rig = self.rig
        for mp in self.plan.meshes:
            mesh = bpy.data.meshes.new(mp.name)
            mesh.from_pydata(mp.vertices, [], mp.faces)
            mesh.validate(clean_customdata=False)
            if mp.uvs:
                uv = mesh.uv_layers.new(name=mp.uv_name or "UVMap")
                flat = [c for uv_co in mp.uvs for c in uv_co[:2]]
                if len(uv.data) * 2 == len(flat):
                    uv.data.foreach_set("uv", flat)
                else:
                    self.warn("mesh %s: UVs dropped after mesh validation changed the topology" % mp.name)
            self._smooth(mesh, mp.sharp_edges)
            obj = bpy.data.objects.new(mp.name, mesh)
            self.collection.objects.link(obj)
            self.mesh_objects[mp.name] = obj
            obj.parent = rig
            if mp.deform:
                for group, items in mp.groups.items():
                    vg = obj.vertex_groups.new(name=group)
                    by_weight = defaultdict(list)
                    for vi, w in items:
                        by_weight[w].append(vi)
                    for w, indices in by_weight.items():
                        vg.add(indices, w, "REPLACE")
                mod = obj.modifiers.new("Armature", "ARMATURE")
                mod.object = rig
            elif mp.parent_bone:
                bone = rig.data.bones[mp.parent_bone]
                obj.parent_type = "BONE"
                obj.parent_bone = mp.parent_bone
                tail = rig.matrix_world @ bone.matrix_local @ Matrix.Translation((0.0, bone.length, 0.0))
                obj.matrix_parent_inverse = tail.inverted()
            if mp.shape_keys:
                self._shape_keys(obj, mp)

    @staticmethod
    def _smooth(mesh, sharp_edges):
        if hasattr(mesh, "shade_smooth"):
            mesh.shade_smooth()
        else:
            mesh.polygons.foreach_set("use_smooth", [True] * len(mesh.polygons))
        if not sharp_edges:
            return
        wanted = {tuple(sorted(e)) for e in sharp_edges}
        flags = [tuple(sorted(e.vertices)) in wanted for e in mesh.edges]
        attr = mesh.attributes.get("sharp_edge") or mesh.attributes.new("sharp_edge", "BOOLEAN", "EDGE")
        attr.data.foreach_set("value", flags)

    @staticmethod
    def _shape_keys(obj, mp):
        obj.shape_key_add(name="Basis", from_mix=False)
        base = [c for v in mp.vertices for c in v]
        for name, value, deltas in mp.shape_keys:
            kb = obj.shape_key_add(name=name, from_mix=False)
            co = list(base)
            for vi, d in deltas.items():
                co[vi * 3] += d[0]
                co[vi * 3 + 1] += d[1]
                co[vi * 3 + 2] += d[2]
            kb.data.foreach_set("co", co)
            kb.slider_min = min(0.0, value)
            kb.slider_max = max(1.0, value)
            kb.value = value
        obj.data.shape_keys.use_relative = True

    # ------------------------------------------------------------------
    def _id(self, id_type, id_name):
        if id_type == "OBJECT":
            return self.rig if id_name is None else bpy.data.objects.get(id_name)
        if id_type == "ARMATURE":
            return self.rig.data
        if id_type == "KEY":
            obj = self.mesh_objects.get(id_name)
            return obj.data.shape_keys if obj is not None else None
        return None

    def _drivers(self):
        for value in self.plan.static_values:
            self._set_static(*value)
        for dp in self.plan.drivers:
            target = self._id(dp.id_type, dp.id_name)
            if target is None:
                self.warn("driver on %s: target data missing, skipped" % dp.label)
                continue
            try:
                fc = target.driver_add(dp.data_path, dp.index) if dp.index >= 0 else target.driver_add(dp.data_path)
            except (TypeError, ValueError) as exc:
                self.warn("driver on %s: %s" % (dp.label, exc))
                continue
            drv = fc.driver
            drv.type = dp.driver_type
            ok = True
            for vp in dp.variables:
                var = drv.variables.new()
                var.name = vp.name
                var.type = vp.kind
                t = var.targets[0]
                if vp.kind == "TRANSFORMS":
                    t.id = self.rig
                    t.bone_target = vp.bone
                    t.transform_type = vp.transform_type
                    t.transform_space = vp.transform_space
                    t.rotation_mode = vp.rotation_mode
                else:
                    src = self._id(vp.id_type, vp.id_name)
                    if src is None:
                        ok = False
                        break
                    t.id_type = vp.id_type
                    t.id = src
                    t.data_path = vp.data_path
            if not ok:
                self.warn("driver on %s: source data missing, skipped" % dp.label)
                if dp.index >= 0:
                    target.driver_remove(dp.data_path, dp.index)
                else:
                    target.driver_remove(dp.data_path)
                continue
            # Blender 4.x seeds new drivers with a Generator modifier, 5.x with two keyframes
            while fc.modifiers:
                fc.modifiers.remove(fc.modifiers[0])
            while fc.keyframe_points:
                fc.keyframe_points.remove(fc.keyframe_points[0], fast=True)
            fc.keyframe_points.add(len(dp.keys))
            for kp, (x, y, interp) in zip(fc.keyframe_points, dp.keys):
                kp.co = (x, y)
                kp.interpolation = interp
                kp.handle_left_type = kp.handle_right_type = "AUTO_CLAMPED"
            fc.extrapolation = dp.extrapolation
            fc.update()

    def _set_static(self, id_type, id_name, path, index, value):
        target = self._id(id_type, id_name)
        if target is None:
            return
        try:
            if index >= 0:
                target.path_resolve(path)[index] = value
            elif path.endswith("]"):
                owner_path, key = path[:path.rindex("[")], path[path.rindex("[") + 2:-2]
                owner = target.path_resolve(owner_path) if owner_path else target
                owner[key] = type(owner[key])(value) if key in owner else value
            else:
                owner_path, attr = path.rsplit(".", 1)
                owner = target.path_resolve(owner_path)
                setattr(owner, attr, type(getattr(owner, attr))(value))
        except (AttributeError, KeyError, TypeError, ValueError) as exc:
            self.warn("cannot set %s: %s" % (path, exc))

    def _log(self):
        text = bpy.data.texts.new(self.plan.name + "_import_log")
        lines = ["Maya rig import: %s" % self.plan.name,
                 "%d bones, %d constraints, %d drivers, %d meshes"
                 % (len(self.plan.bones), len(self.plan.constraints), len(self.plan.drivers), len(self.plan.meshes)),
                 ""]
        lines += ["WARNING: " + w for w in self.warnings] or ["No warnings."]
        text.write("\n".join(lines) + "\n")
