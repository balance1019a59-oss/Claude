"""Turn an interchange file into a Blender-agnostic :class:`RigPlan`.

Nothing in this module touches ``bpy``; :mod:`..builder` only realises the
plan.  The overall mapping is:

* every exported Maya transform (joint, control, group, ikHandle) -> one bone
  in a single armature, so constraints and drivers can stay inside it;
* NURBS curve shapes -> custom-shape widget meshes;
* parent/point/orient/scale/aim constraints -> Armature / Copy* / Track
  constraints, with ``MCH-`` helper bones where Maya's "maintain offset"
  needs one;
* ikHandles (+ poleVector constraints) -> IK constraints with a pole angle
  computed from the rest pose;
* set driven keys, blendWeighted sums and direct connections -> drivers
  whose F-curves carry the keys (no Python expressions, so they evaluate
  even with auto-run scripts disabled);
* skinClusters -> vertex groups + Armature modifier, blendShapes -> shape
  keys.
"""

import math
from dataclasses import dataclass, field
from typing import Optional

from . import mathx as mx
from .convert import (ATTR_CHANNELS, PRIMARY_AXES, SpaceConverter, axis_matrix,
                      blender_rotation_mode, build_channel_maps, dominant_axis, ik_pole_angle)

MAX_NAME = 63
TRACK_ENUM = ("TRACK_X", "TRACK_Y", "TRACK_Z")
TRACK_NEG_ENUM = ("TRACK_NEGATIVE_X", "TRACK_NEGATIVE_Y", "TRACK_NEGATIVE_Z")


def quote(name):
    return '"%s"' % name.replace("\\", "\\\\").replace('"', '\\"')


def pose_path(bone, suffix=""):
    return "pose.bones[%s]%s" % (quote(bone), suffix)


# --- plan data ------------------------------------------------------------

@dataclass
class PropPlan:
    name: str
    value: object
    min: Optional[float] = None
    max: Optional[float] = None
    subtype: Optional[str] = None
    description: str = ""


@dataclass
class BonePlan:
    name: str
    matrix: list
    length: float
    node: Optional[str] = None
    parent: Optional[str] = None
    deform: bool = False
    collection: str = "MCH"
    inherit_scale: str = "FULL"
    rotation_mode: str = "XYZ"
    lock_location: list = field(default_factory=lambda: [False] * 3)
    lock_rotation: list = field(default_factory=lambda: [False] * 3)
    lock_scale: list = field(default_factory=lambda: [False] * 3)
    shape: Optional[tuple] = None
    color: Optional[list] = None
    hide: bool = False
    props: list = field(default_factory=list)


@dataclass
class ConstraintPlan:
    owner: str
    type: str
    name: str
    settings: dict = field(default_factory=dict)
    targets: list = field(default_factory=list)


@dataclass
class VarPlan:
    name: str
    kind: str
    id_type: str = "OBJECT"
    id_name: Optional[str] = None
    data_path: str = ""
    bone: str = ""
    transform_type: str = ""
    transform_space: str = "LOCAL_SPACE"
    rotation_mode: str = "AUTO"


@dataclass
class DriverPlan:
    id_type: str
    id_name: Optional[str]
    data_path: str
    index: int
    variables: list
    keys: list
    extrapolation: str = "LINEAR"
    driver_type: str = "SUM"
    label: str = ""


@dataclass
class MeshPlan:
    name: str
    vertices: list
    faces: list
    uv_name: Optional[str] = None
    uvs: Optional[list] = None
    sharp_edges: list = field(default_factory=list)
    groups: dict = field(default_factory=dict)
    shape_keys: list = field(default_factory=list)
    parent_bone: Optional[str] = None
    deform: bool = False


@dataclass
class RigPlan:
    name: str
    bones: list = field(default_factory=list)
    constraints: list = field(default_factory=list)
    meshes: list = field(default_factory=list)
    drivers: list = field(default_factory=list)
    object_props: list = field(default_factory=list)
    static_values: list = field(default_factory=list)
    warnings: list = field(default_factory=list)

    def bone(self, name):
        for b in self.bones:
            if b.name == name:
                return b
        return None


# --- driver expression sources -------------------------------------------

class _Linear:
    """maya_value = a * sum(vars) + b"""

    def __init__(self, variables, a=1.0, b=0.0):
        self.vars, self.a, self.b = list(variables), a, b


class _Curve:
    """maya_value = curve(var); keys are (x_in_var_domain, y_maya, interp)"""

    def __init__(self, var, keys, extrapolation):
        self.var, self.keys, self.extrapolation = var, keys, extrapolation


class _Const:
    def __init__(self, value):
        self.value = value


def _interp_for(tangent):
    tangent = (tangent or "").lower()
    if tangent in ("step", "stepnext"):
        return "CONSTANT"
    if tangent == "linear":
        return "LINEAR"
    return "BEZIER"


def _eval_keys(keys, x):
    """Piecewise-linear evaluation, used only to fold constant inputs."""
    if x <= keys[0][0]:
        return keys[0][1]
    for (x0, y0, _), (x1, y1, _) in zip(keys, keys[1:]):
        if x <= x1:
            t = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
            return y0 + t * (y1 - y0)
    return keys[-1][1]


# --- planner --------------------------------------------------------------

class RigPlanner:
    def __init__(self, data, scale=0.01, primary_axis="AUTO", name=None):
        if primary_axis != "AUTO" and primary_axis not in PRIMARY_AXES:
            raise ValueError("primary_axis must be AUTO or one of %s" % (PRIMARY_AXES,))
        self.data = data
        self.conv = SpaceConverter(data.get("source", {}).get("up_axis", "y"), scale)
        self.f = self.conv.f
        self.primary_axis = primary_axis
        self.plan = RigPlan(name or data.get("name") or "MayaRig")
        self._warned = set()
        self._prop_counter = 0

    # utilities ---------------------------------------------------------
    def warn(self, msg):
        if msg not in self._warned:
            self._warned.add(msg)
            self.plan.warnings.append(msg)

    def _unique_bone_name(self, wanted, used):
        base = wanted.replace("|", "_")[:MAX_NAME]
        name, i = base, 1
        while name in used:
            suffix = ".%03d" % i
            name = base[:MAX_NAME - len(suffix)] + suffix
            i += 1
        used.add(name)
        return name

    # main --------------------------------------------------------------
    def build(self):
        data = self.data
        self.plan.warnings.extend(data.get("warnings", []))
        self.nodes = {}
        for n in data.get("nodes", []):
            self.nodes[n["name"]] = n
        self.order = self._topological(list(self.nodes))
        used = set()
        self.bone_of = {n: self._unique_bone_name(n, used) for n in self.order}
        self.used_names = used

        self._frames()
        self._axes_and_lengths()
        self._channel_maps()

        self.unparented = set()
        self.helpers = []          # BonePlans created for constraints
        self.constraints = []      # ConstraintPlans for Maya constraints / IK
        self.limit_constraints = []
        self.con_info = {}         # Maya constraint name -> info used by drivers
        self.ik_info = {}          # ikHandle name -> (owner bone, constraint name)
        self.influences = self._skin_influences()

        self._plan_bones()
        self._plan_constraints()
        self._plan_ik()
        self._plan_limits()
        self.plan.bones = self._sorted_bones(self.bone_plans + self.helpers)
        self.plan.constraints = self.limit_constraints + self.constraints
        self._plan_meshes()
        self._plan_drivers()
        return self.plan

    def _topological(self, names):
        out, seen = [], set()

        def visit(n, stack=()):
            if n in seen:
                return
            if n in stack:
                raise ValueError("cycle in hierarchy at %r" % n)
            p = self.nodes[n].get("parent")
            if p in self.nodes:
                visit(p, stack + (n,))
            seen.add(n)
            out.append(n)

        for n in names:
            visit(n)
        return out

    def _sorted_bones(self, bones):
        by_name = {b.name: b for b in bones}
        out, seen = [], set()

        def visit(b):
            if b.name in seen:
                return
            if b.parent in by_name:
                visit(by_name[b.parent])
            seen.add(b.name)
            out.append(b)

        for b in bones:
            visit(b)
        return out

    # frames ------------------------------------------------------------
    def _frames(self):
        self.rot, self.pos, self.parent_rot = {}, {}, {}
        for name in self.order:
            node = self.nodes[name]
            rot, pos, scales, flipped = self.conv.world(node["world_matrix"])
            if flipped:
                self.warn("%s: mirrored (negative scale) transform, Z axis flipped in Blender" % name)
            if any(abs(s - 1.0) > 1e-3 for s in scales):
                self.warn("%s: world scale %s is baked out of the rest pose" %
                          (name, ", ".join("%.3g" % s for s in scales)))
            self.rot[name], self.pos[name] = rot, pos
        for name in self.order:
            node = self.nodes[name]
            parent = node.get("parent")
            if parent in self.rot:
                self.parent_rot[name] = self.rot[parent]
            elif node.get("parent_matrix"):
                self.parent_rot[name] = self.conv.world(node["parent_matrix"])[0]
            else:
                self.parent_rot[name] = mx.matmul(self.conv.C, mx.identity(3))
        pts = list(self.pos.values())
        if pts:
            lo = [min(p[i] for p in pts) for i in range(3)]
            hi = [max(p[i] for p in pts) for i in range(3)]
            diag = mx.length(mx.sub(hi, lo))
        else:
            diag = 0.0
        # fallback bone length: 3% of the rig size, or 10 Maya cm for a single node
        self.default_len = diag * 0.03 if diag > 1e-9 else self.f * 10.0

    def _children(self, name, types=("joint",)):
        return [n for n in self.order
                if self.nodes[n].get("parent") == name and self.nodes[n]["type"] in types]

    def _axes_and_lengths(self):
        self.axis, self.P, self.length = {}, {}, {}
        for name in self.order:
            node = self.nodes[name]
            parent = node.get("parent")
            if node["type"] != "joint":
                self.axis[name] = "Y"
                self.length[name] = self._non_joint_length(node)
            else:
                best = None
                for child in self._children(name):
                    local = mx.mat_vec(mx.transpose(self.rot[name]), mx.sub(self.pos[child], self.pos[name]))
                    dist = mx.length(local)
                    if dist < 1e-6:
                        continue
                    k, s, _exact = dominant_axis(local)
                    score = abs(local[k]) / dist
                    if best is None or score > best[0] + 1e-6:
                        best = (score, k, s, abs(local[k]))
                if self.primary_axis != "AUTO":
                    axis = self.primary_axis
                elif best is not None:
                    axis = ("-" if best[2] < 0 else "") + "XYZ"[best[1]]
                elif parent in self.axis and self.nodes[parent]["type"] == "joint":
                    axis = self.axis[parent]
                else:
                    axis = "X"
                self.axis[name] = axis
                k = "XYZ".index(axis[-1])
                if best is not None and best[1] == k and (best[2] < 0) == axis.startswith("-") and best[3] > 1e-5:
                    self.length[name] = best[3]
                elif parent in self.length and self.nodes[parent]["type"] == "joint":
                    self.length[name] = self.length[parent] * 0.5
                else:
                    self.length[name] = max(node.get("radius", 1.0) * self.f * 2.0, self.default_len)
            self.P[name] = axis_matrix(self.axis[name])

    def _non_joint_length(self, node):
        extent = 0.0
        origin = self.pos[node["name"]]
        for shape in node.get("shapes", []):
            for p in shape.get("points", []):
                extent = max(extent, mx.length(mx.sub(self.conv.point(p), origin)))
        if extent > 0.0:
            return max(extent * 0.5, self.default_len * 0.25)
        return self.default_len

    def _channel_maps(self):
        self.maps, self.bone_rot = {}, {}
        for name in self.order:
            bone_rot = mx.matmul(self.rot[name], self.P[name])
            self.bone_rot[name] = bone_rot
            self.maps[name] = build_channel_maps(self.nodes[name], bone_rot, self.parent_rot[name],
                                                 self.P[name], self.f)

    def bone_matrix(self, name):
        return mx.compose(self.bone_rot[name], self.pos[name])

    # bones -------------------------------------------------------------
    def _skin_influences(self):
        out = set()
        for mesh in self.data.get("meshes", []):
            skin = mesh.get("skin")
            if skin:
                out.update(skin.get("influences", []))
        return out

    def _is_control(self, node):
        return bool(node.get("shapes"))

    def _plan_bones(self):
        any_skin = bool(self.influences)
        self.bone_plans = []
        self.bone_plan_of = {}
        for name in self.order:
            node = self.nodes[name]
            parent = node.get("parent")
            maps = self.maps[name]
            bp = BonePlan(name=self.bone_of[name], matrix=self.bone_matrix(name),
                          length=self.length[name], node=name)
            if parent in self.nodes and node.get("inherits_transform", True):
                bp.parent = self.bone_of[parent]
            elif parent in self.nodes:
                self.warn("%s: inheritsTransform is off, bone left unparented" % name)
            if node["type"] == "joint":
                bp.deform = (name in self.influences) or not any_skin
                bp.collection = "DEF" if bp.deform else "JNT"
                if node.get("segment_scale_compensate", True):
                    bp.inherit_scale = "NONE"
            elif self._is_control(node):
                bp.collection = "CTRL"
            else:
                bp.collection = "MCH"
            bp.rotation_mode = blender_rotation_mode(node.get("rotate_order", "xyz"), maps)
            if bp.rotation_mode == "XYZ" and node.get("rotate_order", "xyz") != "xyz" and \
                    not all(maps["rotate" + a].exact for a in "XYZ"):
                self.warn("%s: rotate order could not be mapped exactly, using XYZ" % name)
            locked = node.get("locked", {})
            for group, attr in (("translate", "lock_location"), ("rotate", "lock_rotation"),
                                ("scale", "lock_scale")):
                flags = locked.get(group, [False] * 3)
                out = getattr(bp, attr)
                for i, axis in enumerate("XYZ"):
                    if flags[i]:
                        out[maps[group + axis].index] = True
            if node.get("shapes"):
                bp.shape = self._shape_mesh(name, node["shapes"])
                bp.color = node.get("color") or next(
                    (s.get("color") for s in node["shapes"] if s.get("color")), None)
            if not node.get("visibility", True):
                bp.hide = True
            for attr in node.get("attributes", []):
                bp.props.append(self._prop_from_attr(name, attr))
            self.bone_plans.append(bp)
            self.bone_plan_of[name] = bp

    def _shape_mesh(self, name, shapes):
        inv = mx.inverse(self.bone_matrix(name))
        verts, edges = [], []
        for shape in shapes:
            pts = shape.get("points", [])
            if len(pts) < 2:
                continue
            start = len(verts)
            for p in pts:
                verts.append(mx.mat_vec(inv, self.conv.point(p) + [1.0])[:3])
            for i in range(len(pts) - 1):
                edges.append((start + i, start + i + 1))
            if shape.get("closed") and len(pts) > 2:
                edges.append((start + len(pts) - 1, start))
        return (verts, edges) if verts else None

    def _attr_scale(self, attr):
        return self.f if attr.get("type") == "linear" else 1.0

    def _prop_from_attr(self, node, attr):
        kind = attr.get("type", "float")
        k = self._attr_scale(attr)
        value = attr.get("value", 0.0)
        desc = "Maya attribute %s.%s" % (node, attr["name"])
        if kind == "bool":
            return PropPlan(attr["name"], bool(value), description=desc)
        if kind in ("int", "enum"):
            prop = PropPlan(attr["name"], int(value), attr.get("min"), attr.get("max"), description=desc)
            if kind == "enum":
                items = attr.get("enum", [])
                prop.description = desc + ": " + ", ".join("%d=%s" % (i, s) for i, s in enumerate(items))
                if prop.min is None:
                    prop.min = 0
                if prop.max is None and items:
                    prop.max = len(items) - 1
            return prop
        lo, hi = attr.get("min"), attr.get("max")
        return PropPlan(attr["name"], float(value) * k,
                        None if lo is None else lo * k, None if hi is None else hi * k,
                        subtype="ANGLE" if kind == "angle" else ("DISTANCE" if kind == "linear" else None),
                        description=desc)

    # helpers -----------------------------------------------------------
    def _helper(self, wanted, matrix, parent, length=None):
        name = self._unique_bone_name("MCH-" + wanted, self.used_names)
        bp = BonePlan(name=name, matrix=matrix, length=length or self.default_len * 0.5,
                      parent=parent, collection="MCH")
        self.helpers.append(bp)
        return bp

    def _resolve_targets(self, con):
        out = []
        for t in con.get("targets", []):
            if t.get("node") in self.nodes:
                out.append((t["node"], float(t.get("weight", 1.0))))
            else:
                self.warn("constraint %s: target %r not exported, ignored" % (con["name"], t.get("node")))
        return out

    @staticmethod
    def _stacked_influences(weights):
        """Influences that make a stack of N copy constraints produce the
        weighted average of their targets."""
        out, running = [], 0.0
        for w in weights:
            running += w
            out.append(0.0 if running <= 0.0 else w / running)
        if out:
            out[0] = 1.0 if weights[0] > 0.0 else 0.0
        return out

    # constraints -------------------------------------------------------
    def _plan_constraints(self):
        for con in self.data.get("constraints", []):
            ctype = con["type"]
            if ctype == "poleVector":
                continue
            owner = con.get("driven")
            if owner not in self.nodes:
                self.warn("constraint %s: driven node %r not exported, skipped" % (con["name"], owner))
                continue
            targets = self._resolve_targets(con)
            if not targets:
                self.warn("constraint %s: no usable targets, skipped" % con["name"])
                continue
            skip = con.get("skip", {})
            if ctype == "parent":
                st, sr = all(skip.get("translate", [False] * 3)), all(skip.get("rotate", [False] * 3))
                if st and sr:
                    continue
                if sr:
                    ctype = "point"
                elif st:
                    ctype = "orient"
                elif any(skip.get("translate", [])) or any(skip.get("rotate", [])):
                    self.warn("constraint %s: partially skipped axes are not supported, "
                              "all axes constrained" % con["name"])
            getattr(self, "_con_" + ctype)(con, owner, targets)

    def _con_parent(self, con, owner, targets):
        bone = self.bone_of[owner]
        self.bone_plan_of[owner].parent = None
        self.unparented.add(owner)
        total = sum(w for _, w in targets) or 1.0
        self.constraints.append(ConstraintPlan(
            bone, "ARMATURE", con["name"],
            targets=[{"subtarget": self.bone_of[t], "weight": w / total} for t, w in targets]))
        self.con_info[con["name"]] = ("armature", bone, con["name"], len(targets))
        if len(targets) > 1:
            self.warn("constraint %s: Blender's Armature constraint does not renormalise "
                      "driven target weights; keep them summing to 1" % con["name"])

    def _world_axes_flags(self, owner, flags, con_name):
        """Map Maya parent-space skip flags to Blender world axes."""
        use = [True, True, True]
        prot = self.parent_rot[owner]
        for i, skipped in enumerate(flags):
            if not skipped:
                continue
            k, _s, exact = dominant_axis(mx.column(prot, i))
            if not exact:
                self.warn("constraint %s: skipped axis is not world aligned in Blender, "
                          "mapped to nearest world axis" % con_name)
            use[k] = False
        return use

    def _stack(self, con, owner, ctype, subtargets, weights, settings):
        bone = self.bone_of[owner]
        names = []
        for i, (sub, infl) in enumerate(zip(subtargets, self._stacked_influences(weights))):
            cname = con["name"] if i == 0 else "%s.%d" % (con["name"], i)
            s = dict(settings)
            s["influence"] = infl
            self.constraints.append(ConstraintPlan(bone, ctype, cname, s, [{"subtarget": sub}]))
            names.append(cname)
        self.con_info[con["name"]] = ("stack", bone, names, len(names))

    def _con_point(self, con, owner, targets):
        subs = []
        for i, (t, _w) in enumerate(targets):
            src = self._helper("%s-pt%d-src" % (self.bone_of[owner], i), self.bone_matrix(t), None)
            self.constraints.append(ConstraintPlan(src.name, "COPY_LOCATION", "follow " + self.bone_of[t],
                                                   {"target_space": "WORLD", "owner_space": "WORLD"},
                                                   [{"subtarget": self.bone_of[t]}]))
            off = self._helper("%s-pt%d" % (self.bone_of[owner], i), self.bone_matrix(owner), src.name)
            subs.append(off.name)
        use = self._world_axes_flags(owner, con.get("skip", {}).get("translate", [False] * 3), con["name"])
        self._stack(con, owner, "COPY_LOCATION", subs, [w for _, w in targets],
                    {"target_space": "WORLD", "owner_space": "WORLD",
                     "use_x": use[0], "use_y": use[1], "use_z": use[2]})

    def _con_orient(self, con, owner, targets):
        subs = []
        for i, (t, _w) in enumerate(targets):
            h = self._helper("%s-or%d" % (self.bone_of[owner], i), self.bone_matrix(owner), self.bone_of[t])
            subs.append(h.name)
        if any(con.get("skip", {}).get("rotate", [])):
            self.warn("constraint %s: skipped rotate axes are not supported, all axes constrained" % con["name"])
        if len(targets) > 1:
            self.warn("constraint %s: multi-target orient blends rotations by stacking, "
                      "results differ slightly from Maya's average" % con["name"])
        self._stack(con, owner, "COPY_ROTATION", subs, [w for _, w in targets],
                    {"target_space": "WORLD", "owner_space": "WORLD", "mix_mode": "REPLACE"})

    def _con_scale(self, con, owner, targets):
        subs = []
        for i, (t, _w) in enumerate(targets):
            h = self._helper("%s-sc%d" % (self.bone_of[owner], i), self.bone_matrix(owner), self.bone_of[t])
            subs.append(h.name)
        use = [True, True, True]
        for i, skipped in enumerate(con.get("skip", {}).get("scale", [False] * 3)):
            if skipped:
                use[self.maps[owner]["scale" + "XYZ"[i]].index] = False
        self._stack(con, owner, "COPY_SCALE", subs, [w for _, w in targets],
                    {"target_space": "WORLD", "owner_space": "WORLD",
                     "use_x": use[0], "use_y": use[1], "use_z": use[2]})

    def _bone_axis(self, owner, vec, what, con_name):
        v = mx.mat_vec(mx.transpose(self.P[owner]), vec)
        k, s, exact = dominant_axis(v)
        if not exact:
            self.warn("constraint %s: %s vector %s is not axis aligned, using nearest axis"
                      % (con_name, what, vec))
        return k, s

    def _con_aim(self, con, owner, targets):
        bone = self.bone_of[owner]
        name = con["name"]
        if len(targets) > 1:
            self.warn("constraint %s: only the first aim target is used" % name)
        target = self.bone_of[targets[0][0]]
        if any(abs(x) > 1e-4 for x in con.get("offset", [0.0, 0.0, 0.0])):
            self.warn("constraint %s: aim offset is not converted" % name)
        ka, sa = self._bone_axis(owner, con.get("aim_vector", [1, 0, 0]), "aim", name)
        ku, su = self._bone_axis(owner, con.get("up_vector", [0, 1, 0]), "up", name)
        track = (TRACK_ENUM if sa > 0 else TRACK_NEG_ENUM)[ka]
        up_type = con.get("world_up_type", "scene")
        up_obj = con.get("world_up_object")
        damped = ConstraintPlan(bone, "DAMPED_TRACK", name, {"track_axis": track}, [{"subtarget": target}])
        self.con_info[name] = ("stack", bone, [name], 1)
        if up_type == "none" or ku == ka:
            self.constraints.append(damped)
            return
        if up_type == "object" and up_obj in self.nodes:
            self.constraints.append(damped)
            self.constraints.append(ConstraintPlan(
                bone, "LOCKED_TRACK", name + " up",
                {"track_axis": (TRACK_ENUM if su > 0 else TRACK_NEG_ENUM)[ku], "lock_axis": "LOCK_" + "XYZ"[ka]},
                [{"subtarget": self.bone_of[up_obj]}]))
            return
        if up_type in ("scene", "vector"):
            world_up = [0.0, 1.0, 0.0] if up_type == "scene" else con.get("world_up_vector", [0, 1, 0])
            wk, ws, wexact = dominant_axis(self.conv.direction(world_up))
            if wk == 2 and wexact and su * ws > 0:
                self.constraints.append(ConstraintPlan(
                    bone, "TRACK_TO", name,
                    {"track_axis": track, "up_axis": "UP_" + "XYZ"[ku], "use_target_z": False},
                    [{"subtarget": target}]))
                return
        self.warn("constraint %s: world up type %r approximated with Damped Track" % (name, up_type))
        self.constraints.append(damped)

    # IK ----------------------------------------------------------------
    def _plan_ik(self):
        poles = {}
        for con in self.data.get("constraints", []):
            if con["type"] == "poleVector":
                t = self._resolve_targets(con)
                if t:
                    poles[con.get("driven")] = t[0][0]
        for ik in self.data.get("ik_handles", []):
            name = ik["name"]
            solver = ik.get("solver", "rp")
            if solver not in ("rp", "sc"):
                self.warn("ikHandle %s: %s solver is not supported, skipped" % (name, solver))
                continue
            start, end = ik.get("start_joint"), ik.get("end_joint")
            if start not in self.nodes or end not in self.nodes:
                self.warn("ikHandle %s: joints not exported, skipped" % name)
                continue
            chain, cur = [], self.nodes[end].get("parent")
            while cur is not None and cur in self.nodes:
                chain.append(cur)
                if cur == start:
                    break
                cur = self.nodes[cur].get("parent")
            if not chain or chain[-1] != start:
                self.warn("ikHandle %s: %s is not an ancestor of %s, skipped" % (name, start, end))
                continue
            if name not in self.nodes:
                self.warn("ikHandle %s: handle transform not exported, skipped" % name)
                continue
            tip = chain[0]
            settings = {"chain_count": len(chain), "use_tail": True, "use_stretch": False,
                        "influence": float(ik.get("ik_blend", 1.0))}
            targets = [{"subtarget": self.bone_of[name]}]
            tip_tail = mx.add(self.pos[tip], mx.scale(mx.column(self.bone_rot[tip], 1), self.length[tip]))
            if mx.length(mx.sub(tip_tail, self.pos[name])) > self.default_len * 1e-3:
                # Blender's IK pulls the tip bone's *tail* onto the target; when the bones do
                # not point down the chain, aim at the rest tail instead so nothing jumps.
                self.warn("ikHandle %s: joints do not point at their children, IK will be approximate "
                          "(try Bone Axis = Auto)" % name)
                h = self._helper("%s-target" % name, mx.compose(self.bone_rot[name], tip_tail), self.bone_of[name])
                targets[0]["subtarget"] = h.name
            pole_node = poles.get(name)
            pole_pos, pole_bone = None, None
            if pole_node is not None:
                pole_pos, pole_bone = self.pos[pole_node], self.bone_of[pole_node]
            elif solver == "rp":
                pv = ik.get("pole_vector", [0.0, 1.0, 0.0])
                start_parent = self.nodes[start].get("parent")
                prot = self.rot[start_parent] if start_parent in self.rot else self.parent_rot[start]
                direction = mx.normalized(mx.mat_vec(prot, pv))
                if mx.length(direction) > 0.5:
                    reach = sum(self.length[j] for j in chain)
                    pole_pos = mx.add(self.pos[start], mx.scale(direction, reach))
                    parent_bone = self.bone_of.get(self.nodes[start].get("parent"))
                    h = self._helper("%s-pole" % name, mx.compose(mx.identity(3), pole_pos), parent_bone)
                    pole_bone = h.name
            if pole_bone is not None:
                base, tip_m = self.bone_matrix(start), self.bone_matrix(tip)
                base_head = self.pos[start]
                base_y = mx.column(base, 1)
                base_tail = mx.add(base_head, mx.scale(base_y[:3], self.length[start]))
                ik_tail = mx.add(self.pos[tip], mx.scale(mx.column(tip_m, 1)[:3], self.length[tip]))
                angle = ik_pole_angle(base_head, base_tail, mx.column(base, 0)[:3], ik_tail, pole_pos)
                twist = float(ik.get("twist", 0.0))
                if abs(twist) > 1e-6:
                    self.warn("ikHandle %s: twist %.1f deg added to the pole angle (approximate)"
                              % (name, math.degrees(twist)))
                    angle += twist
                settings["pole_angle"] = angle
                targets[0]["pole_subtarget"] = pole_bone
            self.constraints.append(ConstraintPlan(self.bone_of[tip], "IK", name, settings, targets))
            self.ik_info[name] = (self.bone_of[tip], name)

    # limits ------------------------------------------------------------
    def _plan_limits(self):
        for name in self.order:
            limits = self.nodes[name].get("limits") or {}
            bone = self.bone_of[name]
            maps = self.maps[name]
            for group, ctype in (("rotate", "LIMIT_ROTATION"), ("translate", "LIMIT_LOCATION"),
                                 ("scale", "LIMIT_SCALE")):
                axes = limits.get(group)
                if not axes:
                    continue
                settings = {"owner_space": "LOCAL", "use_transform_limit": True}
                for i, pair in enumerate(axes):
                    if not pair or (pair[0] is None and pair[1] is None):
                        continue
                    cm = maps[group + "XYZ"[i]]
                    if not cm.exact:
                        self.warn("%s: %s limit axis %s is not bone aligned, approximated"
                                  % (name, group, "XYZ"[i]))
                    lo = None if pair[0] is None else cm.to_blender(pair[0])
                    hi = None if pair[1] is None else cm.to_blender(pair[1])
                    if cm.a < 0:
                        lo, hi = hi, lo
                    axis = "xyz"[cm.index]
                    if ctype == "LIMIT_ROTATION":
                        settings["use_limit_" + axis] = True
                        settings["min_" + axis] = -2.0 * math.pi if lo is None else lo
                        settings["max_" + axis] = 2.0 * math.pi if hi is None else hi
                    else:
                        if lo is not None:
                            settings["use_min_" + axis], settings["min_" + axis] = True, lo
                        if hi is not None:
                            settings["use_max_" + axis], settings["max_" + axis] = True, hi
                if len(settings) > 2:
                    self.limit_constraints.append(ConstraintPlan(bone, ctype, "Maya " + group + " limits",
                                                                 settings))

    # meshes ------------------------------------------------------------
    def _plan_meshes(self):
        self.blendshape_of = {}
        for mesh in self.data.get("meshes", []):
            mp = MeshPlan(mesh["name"], [self.conv.point(v) for v in mesh["vertices"]],
                          [list(f) for f in mesh["faces"]])
            uv = mesh.get("uvs")
            if uv and uv.get("coords") and uv.get("face_uvs"):
                coords = uv["coords"]
                mp.uv_name = uv.get("name", "UVMap")
                mp.uvs = [coords[i] for face in uv["face_uvs"] for i in face]
                if len(mp.uvs) != sum(len(f) for f in mp.faces):
                    self.warn("mesh %s: UV count does not match face corners, UVs skipped" % mp.name)
                    mp.uvs = None
            mp.sharp_edges = [tuple(e) for e in mesh.get("hard_edges", [])]
            skin = mesh.get("skin")
            if skin:
                mp.deform = True
                infl = skin.get("influences", [])
                for vi, pairs in enumerate(skin["weights"]):
                    for idx, w in pairs:
                        node = infl[idx]
                        if node not in self.bone_of:
                            self.warn("mesh %s: influence %s not exported, its weights dropped" % (mp.name, node))
                            continue
                        mp.groups.setdefault(self.bone_of[node], []).append((vi, w))
            elif mesh.get("parent") in self.bone_of:
                mp.parent_bone = self.bone_of[mesh["parent"]]
            used = set()
            for bs in mesh.get("blendshapes", []):
                for t in bs.get("targets", []):
                    key = t["name"]
                    i = 1
                    while key in used:
                        key, i = "%s.%03d" % (t["name"], i), i + 1
                    used.add(key)
                    d = t.get("deltas", {})
                    deltas = {vi: self.conv.delta(off) for vi, off in zip(d.get("indices", []), d.get("offsets", []))}
                    weight = float(t.get("weight", 0.0))
                    mp.shape_keys.append((key, weight, deltas))
                    self.blendshape_of[(bs["node"], t["index"])] = (mp.name, key)
                    if t.get("inbetweens"):
                        self.warn("blendShape %s.%s: in-between targets are not converted" % (bs["node"], t["name"]))
            self.plan.meshes.append(mp)

    # drivers -----------------------------------------------------------
    def _custom_attr(self, node, attr):
        for a in self.nodes.get(node, {}).get("attributes", []):
            if a["name"] == attr:
                return a
        return None

    def _source(self, plug):
        """Return ``(VarPlan, a, b)`` meaning maya_value = a * var + b."""
        if "blendshape" in plug:
            ref = self.blendshape_of.get((plug["blendshape"], plug.get("index")))
            if ref is None:
                return None
            return VarPlan("v", "SINGLE_PROP", "KEY", ref[0], "key_blocks[%s].value" % quote(ref[1])), 1.0, 0.0
        node, attr = plug.get("node"), plug.get("attr")
        if node not in self.nodes:
            return None
        bone = self.bone_of[node]
        if attr in ATTR_CHANNELS:
            cm = self.maps[node][attr]
            if not cm.exact:
                self.warn("driver source %s.%s is not bone aligned, approximated" % (node, attr))
            return (VarPlan("v", "TRANSFORMS", bone=bone, transform_type=cm.transform_type),
                    1.0 / cm.a, -cm.b / cm.a)
        if attr == "visibility":
            return VarPlan("v", "SINGLE_PROP", "ARMATURE", None, "bones[%s].hide" % quote(bone)), -1.0, 1.0
        custom = self._custom_attr(node, attr)
        if custom is not None:
            return (VarPlan("v", "SINGLE_PROP", data_path=pose_path(bone, "[%s]" % quote(attr))),
                    1.0 / self._attr_scale(custom), 0.0)
        return None

    def _target(self, tgt):
        """Return ``(id_type, id_name, data_path, index, a, b)`` with
        blender = a * maya + b, or None."""
        if "blendshape" in tgt:
            ref = self.blendshape_of.get((tgt["blendshape"], tgt.get("index")))
            if ref is None:
                return None
            return "KEY", ref[0], "key_blocks[%s].value" % quote(ref[1]), -1, 1.0, 0.0
        if "constraint" in tgt:
            info = self.con_info.get(tgt["constraint"])
            if info is None:
                return None
            kind, bone, names, count = info
            i = int(tgt.get("index", 0))
            if i >= count:
                return None
            if kind == "armature":
                return ("OBJECT", None, pose_path(bone, ".constraints[%s].targets[%d].weight" % (quote(names), i)),
                        -1, 1.0, 0.0)
            if i == 0:
                self.warn("constraint %s: driven weight of the first target is not converted "
                          "(stacked constraints)" % tgt["constraint"])
                return None
            self.warn("constraint %s: driven target weights mapped to constraint influence "
                      "(exact only when weights sum to 1)" % tgt["constraint"])
            return "OBJECT", None, pose_path(bone, ".constraints[%s].influence" % quote(names[i])), -1, 1.0, 0.0
        node, attr = tgt.get("node"), tgt.get("attr")
        if node not in self.nodes:
            return None
        bone = self.bone_of[node]
        if attr in ATTR_CHANNELS:
            cm = self.maps[node][attr]
            if not cm.exact:
                self.warn("driven channel %s.%s is not bone aligned, approximated" % (node, attr))
            return "OBJECT", None, pose_path(bone, "." + cm.data_path), cm.index, cm.a, cm.b
        if attr == "visibility":
            return "ARMATURE", None, "bones[%s].hide" % quote(bone), -1, -1.0, 1.0
        if attr == "ikBlend" and node in self.ik_info:
            owner, cname = self.ik_info[node]
            return "OBJECT", None, pose_path(owner, ".constraints[%s].influence" % quote(cname)), -1, 1.0, 0.0
        custom = self._custom_attr(node, attr)
        if custom is not None:
            return "OBJECT", None, pose_path(bone, "[%s]" % quote(attr)), -1, self._attr_scale(custom), 0.0
        return None

    def _describe(self, plug):
        if "blendshape" in plug:
            return "%s.weight[%s]" % (plug["blendshape"], plug.get("index"))
        if "constraint" in plug:
            return "%s.target[%s].weight" % (plug["constraint"], plug.get("index"))
        return "%s.%s" % (plug.get("node"), plug.get("attr"))

    def _plan_drivers(self):
        for drv in self.data.get("drivers", []):
            label = self._describe(drv["target"])
            dest = self._target(drv["target"])
            if dest is None:
                self.warn("driver on %s: destination not available in Blender, skipped" % label)
                continue
            try:
                source = self._compile(drv["expr"], label)
            except _Unsupported as exc:
                self.warn("driver on %s: %s, skipped" % (label, exc))
                continue
            self._finalize(dest, source, label)

    def _compile(self, expr, label):
        op = expr.get("op")
        if op == "plug":
            src = self._source(expr)
            if src is None:
                raise _Unsupported("source %s not available" % self._describe(expr))
            var, a, b = src
            return _Linear([var], a, b)
        if op == "const":
            return _Const(float(expr["value"]))
        if op == "affine":
            inner = self._compile(expr["input"], label)
            k, c = float(expr.get("scale", 1.0)), float(expr.get("offset", 0.0))
            if isinstance(inner, _Const):
                return _Const(k * inner.value + c)
            if isinstance(inner, _Linear):
                return _Linear(inner.vars, k * inner.a, k * inner.b + c)
            return _Curve(inner.var, [(x, k * y + c, i) for x, y, i in inner.keys], inner.extrapolation)
        if op == "curve":
            return self._compile_curve(expr, label)
        if op == "sum":
            weights = expr.get("weights") or [1.0] * len(expr["inputs"])
            variables, const = [], 0.0
            for sub, w in zip(expr["inputs"], weights):
                term = self._compile({"op": "affine", "scale": w, "offset": 0.0, "input": sub}, label)
                if isinstance(term, _Const):
                    const += term.value
                elif isinstance(term, _Linear) and abs(term.a - 1.0) < 1e-12:
                    variables.extend(term.vars)
                    const += term.b
                else:
                    variables.append(self._materialize(term, label))
            if not variables:
                return _Const(const)
            return _Linear(variables, 1.0, const)
        raise _Unsupported("unsupported node %r" % expr.get("node_type", op))

    def _compile_curve(self, expr, label):
        keys = [(float(k[0]), float(k[1]), _interp_for(k[3] if len(k) > 3 else "")) for k in expr["keys"]]
        keys.sort(key=lambda k: k[0])
        if not keys:
            raise _Unsupported("empty animation curve")
        pre, post = expr.get("pre", "constant"), expr.get("post", "constant")
        extrap = "LINEAR" if "linear" in (pre, post) else "CONSTANT"
        if pre != post or pre not in ("constant", "linear"):
            self.warn("driver on %s: curve infinity %s/%s approximated as %s" % (label, pre, post, extrap.lower()))
        inner = self._compile(expr["input"], label)
        if isinstance(inner, _Const):
            return _Const(_eval_keys(keys, inner.value))
        if isinstance(inner, _Linear) and len(inner.vars) == 1:
            var, a, b = inner.vars[0], inner.a, inner.b
        else:
            var, a, b = self._materialize(inner, label), 1.0, 0.0
        mapped = sorted(((x - b) / a, y, interp) for x, y, interp in keys)
        if a < 0:
            # segment interpolation belongs to the left key; shift it after reversing
            interps = [k[2] for k in sorted(keys, key=lambda k: -k[0])]
            mapped = [(x, y, interps[i + 1] if i + 1 < len(interps) else interps[-1])
                      for i, (x, y, _) in enumerate(mapped)]
        return _Curve(var, mapped, extrap)

    def _materialize(self, source, label):
        self._prop_counter += 1
        name = "m2b_%03d" % self._prop_counter
        self.plan.object_props.append(PropPlan(name, 0.0, description="intermediate value for " + label))
        self._finalize(("OBJECT", None, '[%s]' % quote(name), -1, 1.0, 0.0), source, label)
        return VarPlan("v", "SINGLE_PROP", data_path='[%s]' % quote(name))

    def _finalize(self, dest, source, label):
        id_type, id_name, path, index, da, db = dest
        if isinstance(source, _Const):
            self.plan.static_values.append((id_type, id_name, path, index, da * source.value + db))
            return
        if isinstance(source, _Linear):
            variables = source.vars
            keys = [(0.0, da * source.b + db, "LINEAR"), (1.0, da * (source.a + source.b) + db, "LINEAR")]
            extrap, dtype = "LINEAR", "SUM"
        else:
            variables = [source.var]
            keys = [(x, da * y + db, interp) for x, y, interp in source.keys]
            extrap, dtype = source.extrapolation, "AVERAGE"
        named = []
        for i, v in enumerate(variables):
            v = VarPlan(**{**v.__dict__, "name": "v%d" % i})
            named.append(v)
        self.plan.drivers.append(DriverPlan(id_type, id_name, path, index, named, keys, extrap, dtype, label))


class _Unsupported(Exception):
    pass


def make_plan(data, **options):
    return RigPlanner(data, **options).build()
