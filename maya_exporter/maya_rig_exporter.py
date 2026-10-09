"""Export a Maya rig to the maya2blender interchange format (.m2b.json).

Inside Maya (Script Editor, Python tab)::

    import maya_rig_exporter
    maya_rig_exporter.show_ui()                                   # small window
    maya_rig_exporter.export_rig("D:/out/hero.m2b.json")          # selected root(s)
    maya_rig_exporter.export_rig("D:/out/hero.m2b.json", roots=["rig_grp", "geo_grp"])

Batch (mayapy)::

    mayapy maya_rig_exporter.py scene.ma out.m2b.json --root rig_grp --root geo_grp

The rig should be in its bind / default pose: that pose becomes the Blender
rest pose.  Everything is written in Maya internal units (cm, radians).
"""

import json
import re
import sys

try:  # the pure helpers below are importable (and tested) outside Maya
    import maya.api.OpenMaya as om
    import maya.api.OpenMayaAnim as oma
    from maya import cmds
except ImportError:  # pragma: no cover - only hit outside Maya
    om = oma = cmds = None

FORMAT_NAME = "maya2blender-rig"
FORMAT_VERSION = 1
EXPORTER_VERSION = "0.1.0"

ROTATE_ORDERS = ("xyz", "yzx", "zxy", "xzy", "yxz", "zyx")
CHANNELS = ("translateX", "translateY", "translateZ", "rotateX", "rotateY", "rotateZ",
            "scaleX", "scaleY", "scaleZ")
CONSTRAINT_TYPES = {
    "parentConstraint": "parent", "pointConstraint": "point", "orientConstraint": "orient",
    "scaleConstraint": "scale", "aimConstraint": "aim", "poleVectorConstraint": "poleVector",
}
WORLD_UP_TYPES = ("scene", "object", "objectrotation", "vector", "none")
SKIP_NODE_TYPES = ("ikEffector",)


# --- pure helpers -----------------------------------------------------------

def expand_components(items):
    """``["vtx[0:2]", "vtx[5]"]`` -> ``[0, 1, 2, 5]``"""
    out = []
    for item in items or []:
        m = re.search(r"\[(\d+)(?::(\d+))?\]", item)
        if not m:
            continue
        lo = int(m.group(1))
        hi = int(m.group(2)) if m.group(2) else lo
        out.extend(range(lo, hi + 1))
    return out


def round_floats(obj, digits=6):
    if isinstance(obj, float):
        r = round(obj, digits)
        return 0.0 if r == 0.0 else r
    if isinstance(obj, dict):
        return {k: round_floats(v, digits) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [round_floats(v, digits) for v in obj]
    return obj


def clean_enum_names(spec):
    """attributeQuery listEnum string ``"Off:On=3"`` -> ``["Off", "On"]``"""
    return [part.split("=")[0] for part in (spec or "").split(":") if part]


def strip_namespace(name):
    return name.split("|")[-1].split(":")[-1]


# --- Maya helpers -----------------------------------------------------------

def _dag(name):
    sel = om.MSelectionList()
    sel.add(name)
    return sel.getDagPath(0)


def _mobject(name):
    sel = om.MSelectionList()
    sel.add(name)
    return sel.getDependNode(0)


def _flat(matrix):
    return [matrix.getElement(r, c) for r in range(4) for c in range(4)]


def _long(name):
    found = cmds.ls(name, long=True)
    return found[0] if found else name


def _inputs(plug, skip_conversion=False):
    return cmds.listConnections(plug, source=True, destination=False, plugs=True,
                                skipConversionNodes=skip_conversion) or []


def _node_of(plug):
    return plug.split(".", 1)[0]


def _attr_of(plug):
    return plug.split(".", 1)[1]


class RigExporter(object):
    def __init__(self, roots, strip_namespaces=True, curve_samples=8, weight_threshold=1e-5,
                 include_meshes=True):
        self.roots = [_long(r) for r in roots]
        self.strip_namespaces = strip_namespaces
        self.curve_samples = curve_samples
        self.weight_threshold = weight_threshold
        self.include_meshes = include_meshes
        self.warnings = []
        self.lin = om.MDistance.uiToInternal(1.0)   # UI linear unit -> cm
        self.ang = om.MAngle.uiToInternal(1.0)      # UI angle unit -> radians

    def warn(self, msg):
        if msg not in self.warnings:
            self.warnings.append(msg)

    # ------------------------------------------------------------------
    def run(self):
        transforms = self._collect_transforms()
        mesh_shapes = self._collect_meshes() if self.include_meshes else []
        for shape in mesh_shapes:
            for infl in self._skin_influences(shape):
                transforms.add(infl)
                parent = cmds.listRelatives(infl, parent=True, fullPath=True)
                while parent:
                    transforms.add(parent[0])
                    parent = cmds.listRelatives(parent[0], parent=True, fullPath=True)
        transforms = self._prune(transforms)
        self.order = sorted(transforms, key=lambda n: n.count("|"))
        self.names = self._export_names(self.order)
        self.included = set(self.order)
        self.custom_attrs = {}

        nodes = [self._node(n) for n in self.order]
        meshes = [m for m in (self._mesh(s) for s in mesh_shapes) if m]
        constraints = self._constraints()
        ik_handles = [self._ik(n) for n in self.order if cmds.nodeType(n) == "ikHandle"]
        drivers = self._drivers(meshes)
        scene = cmds.file(q=True, sceneName=True) or "untitled"
        name = strip_namespace(self.roots[0]) if self.roots else "rig"
        return round_floats({
            "format": FORMAT_NAME,
            "version": FORMAT_VERSION,
            "name": name,
            "source": {
                "application": "maya", "maya_version": cmds.about(version=True),
                "exporter_version": EXPORTER_VERSION, "scene": scene,
                "up_axis": cmds.upAxis(q=True, axis=True),
                "ui_linear_unit": cmds.currentUnit(q=True, linear=True),
            },
            "units": {"linear": "cm", "angle": "rad"},
            "nodes": nodes,
            "meshes": meshes,
            "constraints": [c for c in constraints if c],
            "ik_handles": [i for i in ik_handles if i],
            "drivers": drivers,
            "warnings": self.warnings,
        })

    # collection -------------------------------------------------------
    def _is_rig_transform(self, node):
        ntype = cmds.nodeType(node)
        if ntype in SKIP_NODE_TYPES or ntype in CONSTRAINT_TYPES or "Constraint" in ntype:
            return False
        shapes = cmds.listRelatives(node, shapes=True, noIntermediate=True, fullPath=True) or []
        types = {cmds.nodeType(s) for s in shapes}
        if types & {"mesh", "nurbsSurface", "camera", "subdiv"}:
            return False
        return True

    def _collect_transforms(self):
        out = set()
        for root in self.roots:
            nodes = [root] + (cmds.listRelatives(root, allDescendents=True, type="transform", fullPath=True) or [])
            for n in nodes:
                if self._is_rig_transform(n):
                    out.add(n)
        return out

    def _referenced(self, node):
        outs = cmds.listConnections(node, source=False, destination=True) or []
        return any(cmds.nodeType(o).startswith("animCurveU") or "Constraint" in cmds.nodeType(o)
                   or cmds.nodeType(o) in ("unitConversion", "blendWeighted", "ikHandle") for o in outs)

    def _prune(self, transforms):
        """Drop plain groups that hold no rig node and drive nothing."""
        keep = set(transforms)
        for n in sorted(transforms, key=lambda x: -x.count("|")):
            if cmds.nodeType(n) != "transform":
                continue
            if cmds.listRelatives(n, shapes=True, noIntermediate=True, type="nurbsCurve"):
                continue
            children = cmds.listRelatives(n, children=True, type="transform", fullPath=True) or []
            if any(c in keep for c in children) or self._referenced(n):
                continue
            if cmds.listRelatives(n, shapes=True, type="locator"):
                continue
            keep.discard(n)
        return keep

    def _collect_meshes(self):
        shapes = set()
        for root in self.roots:
            for s in cmds.listRelatives(root, allDescendents=True, type="mesh", fullPath=True) or []:
                if not cmds.getAttr(s + ".intermediateObject"):
                    shapes.add(s)
        return sorted(shapes)

    def _export_names(self, nodes):
        names, used = {}, set()
        for n in nodes:
            base = strip_namespace(n) if self.strip_namespaces else cmds.ls(n)[0]
            name, i = base, 1
            while name in used:
                name, i = "%s_%d" % (base, i), i + 1
            if name != base:
                self.warn("duplicate name %s renamed to %s" % (base, name))
            used.add(name)
            names[n] = name
        return names

    def name(self, node):
        return self.names.get(_long(node))

    # nodes ------------------------------------------------------------
    def _vec(self, plug, factor=1.0):
        return [v * factor for v in cmds.getAttr(plug)[0]]

    def _node(self, n):
        ntype = cmds.nodeType(n)
        dag = _dag(n)
        parent = cmds.listRelatives(n, parent=True, fullPath=True)
        rec = {
            "name": self.names[n],
            "type": ntype if ntype in ("joint", "ikHandle") else "transform",
            "parent": self.names.get(parent[0]) if parent else None,
            "world_matrix": _flat(dag.inclusiveMatrix()),
            "parent_matrix": _flat(dag.exclusiveMatrix()),
            "translate": self._vec(n + ".translate", self.lin),
            "rotate": self._vec(n + ".rotate", self.ang),
            "scale": self._vec(n + ".scale"),
            "rotate_order": ROTATE_ORDERS[cmds.getAttr(n + ".rotateOrder")],
            "rotate_axis": self._vec(n + ".rotateAxis", self.ang),
            "inherits_transform": bool(cmds.getAttr(n + ".inheritsTransform")),
            "visibility": bool(cmds.getAttr(n + ".visibility")),
        }
        if ntype == "joint":
            rec["joint_orient"] = self._vec(n + ".jointOrient", self.ang)
            rec["segment_scale_compensate"] = bool(cmds.getAttr(n + ".segmentScaleCompensate"))
            rec["radius"] = cmds.getAttr(n + ".radius")
        for pivot in ("rotatePivot", "scalePivot"):
            if any(abs(v) > 1e-4 for v in cmds.getAttr("%s.%s" % (n, pivot))[0]):
                self.warn("%s: non-zero %s is ignored (rotations happen about the node origin)"
                          % (rec["name"], pivot))
        rec["locked"] = {g: [self._is_locked(n + "." + g + a) for a in "XYZ"]
                         for g in ("translate", "rotate", "scale")}
        limits = self._limits(n)
        if limits:
            rec["limits"] = limits
        shapes = self._curve_shapes(n)
        if shapes:
            rec["shapes"] = shapes
            rec["color"] = self._color(n)
        attrs = self._attributes(n)
        self.custom_attrs[n] = [a["name"] for a in attrs]
        if attrs:
            rec["attributes"] = attrs
        return rec

    @staticmethod
    def _is_locked(plug):
        return bool(cmds.getAttr(plug, lock=True) or not (cmds.getAttr(plug, keyable=True)
                                                          or cmds.getAttr(plug, channelBox=True)))

    def _limits(self, n):
        out = {}
        for group, flag, factor in (("translate", "t", self.lin), ("rotate", "r", self.ang), ("scale", "s", 1.0)):
            axes, any_on = [], False
            for a in "xyz":
                enabled = cmds.transformLimits(n, q=True, **{"e" + flag + a: True})
                values = cmds.transformLimits(n, q=True, **{flag + a: True})
                lo = values[0] * factor if enabled[0] else None
                hi = values[1] * factor if enabled[1] else None
                any_on = any_on or enabled[0] or enabled[1]
                axes.append([lo, hi])
            if any_on:
                out[group] = axes
        return out

    def _color(self, node):
        for target in (cmds.listRelatives(node, shapes=True, fullPath=True) or []) + [node]:
            if not cmds.getAttr(target + ".overrideEnabled"):
                continue
            if cmds.attributeQuery("overrideRGBColors", node=target, exists=True) and \
                    cmds.getAttr(target + ".overrideRGBColors"):
                return list(cmds.getAttr(target + ".overrideColorRGB")[0])
            idx = cmds.getAttr(target + ".overrideColor")
            if idx > 0:
                return list(cmds.colorIndex(idx, q=True))
        return None

    def _curve_shapes(self, node):
        out = []
        for s in cmds.listRelatives(node, shapes=True, noIntermediate=True, type="nurbsCurve", fullPath=True) or []:
            fn = om.MFnNurbsCurve(_dag(s))
            closed = fn.form in (om.MFnNurbsCurve.kPeriodic, om.MFnNurbsCurve.kClosed)
            if fn.degree == 1:
                pts = [[p.x, p.y, p.z] for p in fn.cvPositions(om.MSpace.kWorld)]
                if closed and len(pts) > 2 and all(abs(a - b) < 1e-6 for a, b in zip(pts[0], pts[-1])):
                    pts.pop()
            else:
                start, end = fn.knotDomain
                count = max(4, fn.numSpans * self.curve_samples)
                steps = count if closed else count - 1
                pts = []
                for i in range(count):
                    p = fn.getPointAtParam(start + (end - start) * i / float(steps), om.MSpace.kWorld)
                    pts.append([p.x, p.y, p.z])
            out.append({"points": pts, "closed": closed, "color": self._color_of_shape(s)})
        return out

    def _color_of_shape(self, shape):
        if not cmds.getAttr(shape + ".overrideEnabled"):
            return None
        if cmds.getAttr(shape + ".overrideRGBColors"):
            return list(cmds.getAttr(shape + ".overrideColorRGB")[0])
        idx = cmds.getAttr(shape + ".overrideColor")
        return list(cmds.colorIndex(idx, q=True)) if idx > 0 else None

    def _attributes(self, n):
        out = []
        names = (cmds.listAttr(n, userDefined=True, keyable=True) or []) + \
                (cmds.listAttr(n, userDefined=True, channelBox=True) or [])
        for attr in dict.fromkeys(names):
            plug = "%s.%s" % (n, attr)
            atype = cmds.getAttr(plug, type=True)
            if atype in ("double", "float"):
                kind, factor = "float", 1.0
            elif atype == "doubleLinear":
                kind, factor = "linear", self.lin
            elif atype == "doubleAngle":
                kind, factor = "angle", self.ang
            elif atype in ("long", "short", "byte"):
                kind, factor = "int", 1.0
            elif atype == "bool":
                kind, factor = "bool", 1.0
            elif atype == "enum":
                kind, factor = "enum", 1.0
            else:
                continue
            rec = {"name": attr, "type": kind, "value": cmds.getAttr(plug) * factor
                   if kind not in ("bool",) else bool(cmds.getAttr(plug))}
            if kind == "enum":
                rec["enum"] = clean_enum_names((cmds.attributeQuery(attr, node=n, listEnum=True) or [""])[0])
            elif kind != "bool":
                if cmds.attributeQuery(attr, node=n, minExists=True):
                    rec["min"] = cmds.attributeQuery(attr, node=n, minimum=True)[0] * factor
                if cmds.attributeQuery(attr, node=n, maxExists=True):
                    rec["max"] = cmds.attributeQuery(attr, node=n, maximum=True)[0] * factor
            out.append(rec)
        return out

    # meshes -----------------------------------------------------------
    def _history(self, shape, node_type):
        return cmds.ls(cmds.listHistory(shape, pruneDagObjects=True) or [], type=node_type) or []

    def _skin_influences(self, shape):
        out = []
        for sc in self._history(shape, "skinCluster"):
            out.extend(_long(i) for i in cmds.skinCluster(sc, q=True, influence=True) or [])
        return out

    def _mesh(self, shape):
        transform = cmds.listRelatives(shape, parent=True, fullPath=True)[0]
        dag = _dag(shape)
        fn = om.MFnMesh(dag)
        points = fn.getPoints(om.MSpace.kWorld)
        counts, connects = fn.getVertices()
        faces, k = [], 0
        for c in counts:
            faces.append(list(connects[k:k + c]))
            k += c
        rec = {
            "name": strip_namespace(transform) if self.strip_namespaces else cmds.ls(transform)[0],
            "vertices": [[p.x, p.y, p.z] for p in points],
            "faces": faces,
            "hard_edges": [list(fn.getEdgeVertices(i)) for i in range(fn.numEdges) if not fn.isEdgeSmooth(i)],
        }
        uv_set = fn.currentUVSetName()
        if uv_set:
            us, vs = fn.getUVs(uv_set)
            uv_counts, uv_ids = fn.getAssignedUVs(uv_set)
            if len(uv_ids) == len(connects):
                face_uvs, k = [], 0
                for c in uv_counts:
                    face_uvs.append(list(uv_ids[k:k + c]))
                    k += c
                rec["uvs"] = {"name": uv_set, "coords": [[u, v] for u, v in zip(us, vs)], "face_uvs": face_uvs}
            else:
                self.warn("mesh %s: some faces have no UVs, UVs not exported" % rec["name"])
        skins = self._history(shape, "skinCluster")
        if skins:
            rec["skin"] = self._skin(skins[0], dag, len(points), rec["name"])
            if len(skins) > 1:
                self.warn("mesh %s: only the first of %d skinClusters is exported" % (rec["name"], len(skins)))
        else:
            parent = cmds.listRelatives(transform, parent=True, fullPath=True)
            rec["parent"] = self.names.get(parent[0]) if parent else None
        blendshapes = [self._blendshape(bs, transform, rec["name"]) for bs in self._history(shape, "blendShape")]
        if blendshapes:
            rec["blendshapes"] = blendshapes
        return rec

    def _skin(self, sc, shape_dag, count, mesh_name):
        fn = oma.MFnSkinCluster(_mobject(sc))
        infl_paths = fn.influenceObjects()
        names = []
        for p in infl_paths:
            exported = self.names.get(p.fullPathName())
            names.append(exported or p.partialPathName())
            bpm_plug = fn.findPlug("bindPreMatrix", False).elementByLogicalIndex(fn.indexForInfluenceObject(p))
            bind = om.MFnMatrixData(bpm_plug.asMObject()).matrix()
            if not (p.inclusiveMatrix() * bind).isEquivalent(om.MMatrix(), 1e-3):
                self.warn("%s: influence %s is not at its bind pose; the Blender rest pose will "
                          "differ from the skin bind (use Skin > Go to Bind Pose before exporting)"
                          % (mesh_name, p.partialPathName()))
        comp = om.MFnSingleIndexedComponent()
        comp_obj = comp.create(om.MFn.kMeshVertComponent)
        comp.setCompleteData(count)
        weights, n = fn.getWeights(shape_dag, comp_obj)
        per_vertex = []
        for v in range(count):
            row = [[i, weights[v * n + i]] for i in range(n) if weights[v * n + i] > self.weight_threshold]
            total = sum(w for _, w in row) or 1.0
            per_vertex.append([[i, w / total] for i, w in row])
        return {"node": sc, "influences": names, "weights": per_vertex}

    def _orig_points(self, transform):
        for s in cmds.listRelatives(transform, shapes=True, fullPath=True) or []:
            if cmds.getAttr(s + ".intermediateObject") and not _inputs(s + ".inMesh"):
                return om.MFnMesh(_dag(s)).getPoints(om.MSpace.kObject)
        return None

    def _blendshape(self, bs, transform, mesh_name):
        world = _dag(transform).inclusiveMatrix()
        aliases = cmds.aliasAttr(bs, q=True) or []
        alias_of = {aliases[i + 1]: aliases[i] for i in range(0, len(aliases), 2)}
        targets = []
        for idx in cmds.getAttr(bs + ".weight", multiIndices=True) or []:
            name = alias_of.get("weight[%d]" % idx, "target%d" % idx)
            group = "%s.inputTarget[0].inputTargetGroup[%d]" % (bs, idx)
            items = cmds.getAttr(group + ".inputTargetItem", multiIndices=True) or [6000]
            item = "%s.inputTargetItem[6000]" % group
            indices, offsets = [], []
            live = _inputs(item + ".inputGeomTarget")
            if live:
                orig = self._orig_points(transform)
                target_pts = om.MFnMesh(_dag(_node_of(live[0]))).getPoints(om.MSpace.kObject)
                if orig is not None and len(orig) == len(target_pts):
                    for i, (a, b) in enumerate(zip(orig, target_pts)):
                        d = b - a
                        if d.length() > 1e-6:
                            indices.append(i)
                            offsets.append(d)
                else:
                    self.warn("blendShape %s.%s: live target topology differs, skipped" % (bs, name))
            else:
                pts = cmds.getAttr(item + ".inputPointsTarget") or []
                indices = expand_components(cmds.getAttr(item + ".inputComponentsTarget") or [])
                offsets = [om.MVector(p[0], p[1], p[2]) * self.lin for p in pts]
                if len(indices) != len(offsets):
                    self.warn("blendShape %s.%s: component/point mismatch, skipped" % (bs, name))
                    indices, offsets = [], []
            world_offsets = [list(o * world) for o in offsets]   # direction only (row vector, w=0)
            targets.append({
                "name": name, "index": idx, "weight": cmds.getAttr("%s.weight[%d]" % (bs, idx)),
                "deltas": {"indices": indices, "offsets": world_offsets},
                "inbetweens": [i for i in items if i != 6000],
            })
        self.blendshape_nodes = getattr(self, "blendshape_nodes", {})
        self.blendshape_nodes[bs] = {v: int(k[7:-1]) for k, v in alias_of.items() if k.startswith("weight[")}
        return {"node": bs, "mesh": mesh_name, "targets": targets}

    # constraints / IK ---------------------------------------------------
    def _constraint_nodes(self):
        found = []
        for n in self.order:
            for c in cmds.listConnections(n, source=True, destination=False) or []:
                if "Constraint" in cmds.nodeType(c) and c not in found:
                    found.append(c)
        return found

    def _constraints(self):
        out = []
        self.constraint_names = {}
        for c in self._constraint_nodes():
            ctype = cmds.nodeType(c)
            kind = CONSTRAINT_TYPES.get(ctype)
            if kind is None:
                self.warn("%s: %s is not supported" % (c, ctype))
                continue
            func = getattr(cmds, ctype)
            driven_src = _inputs(c + ".constraintParentInverseMatrix")
            driven = self.name(_node_of(driven_src[0])) if driven_src else None
            if driven is None:
                outs = [o for o in cmds.listConnections(c, source=False, destination=True) or []
                        if self.name(o) and o != c]
                driven = self.name(outs[0]) if outs else None
            if driven is None:
                continue
            targets = func(c, q=True, targetList=True) or []
            aliases = func(c, q=True, weightAliasList=True) or []
            rec = {"name": strip_namespace(c), "type": kind, "driven": driven,
                   "targets": [{"node": self.name(t) or strip_namespace(t), "weight": cmds.getAttr(c + "." + a)}
                               for t, a in zip(targets, aliases)]}
            self.constraint_names[c] = (rec["name"], aliases)
            if kind in ("parent", "point", "orient", "scale"):
                driven_long = _long(_node_of(driven_src[0])) if driven_src else None
                rec["skip"] = self._skips(c, driven_long)
            if kind == "aim":
                rec["aim_vector"] = list(cmds.getAttr(c + ".aimVector")[0])
                rec["up_vector"] = list(cmds.getAttr(c + ".upVector")[0])
                rec["world_up_type"] = WORLD_UP_TYPES[cmds.getAttr(c + ".worldUpType")]
                rec["world_up_vector"] = list(cmds.getAttr(c + ".worldUpVector")[0])
                up = _inputs(c + ".worldUpMatrix")
                rec["world_up_object"] = self.name(_node_of(up[0])) if up else None
                rec["offset"] = [v * self.ang for v in cmds.getAttr(c + ".offset")[0]]
            out.append(rec)
        return out

    def _skips(self, c, driven):
        skip = {}
        for group in ("translate", "rotate", "scale"):
            flags = []
            for a in "XYZ":
                if driven is None:
                    flags.append(False)
                    continue
                src = [_node_of(p) for p in _inputs("%s.%s%s" % (driven, group, a), skip_conversion=True)]
                flags.append(c not in src)
            skip[group] = flags
        return skip

    def _ik(self, h):
        solver = cmds.ikHandle(h, q=True, solver=True) or "ikRPsolver"
        start = cmds.ikHandle(h, q=True, startJoint=True)
        effector = cmds.ikHandle(h, q=True, endEffector=True)
        end_src = _inputs(effector + ".translateX", skip_conversion=True) if effector else []
        end = _node_of(end_src[0]) if end_src else None
        return {
            "name": self.names[h],
            "solver": {"ikRPsolver": "rp", "ikSCsolver": "sc", "ikSplineSolver": "spline"}.get(solver, solver),
            "start_joint": self.name(start),
            "end_joint": self.name(end) if end else None,
            "pole_vector": list(cmds.getAttr(h + ".poleVector")[0]),
            "twist": cmds.getAttr(h + ".twist") * self.ang,
            "ik_blend": cmds.getAttr(h + ".ikBlend"),
        }

    # drivers ------------------------------------------------------------
    def _drivers(self, meshes):
        dests = []
        for n in self.order:
            name = self.names[n]
            attrs = list(CHANNELS) + ["visibility"]
            attrs += self.custom_attrs.get(n, [])
            if cmds.nodeType(n) == "ikHandle":
                attrs.append("ikBlend")
            for attr in attrs:
                dests.append(("%s.%s" % (n, attr), {"node": name, "attr": attr}))
        for bs, by_alias in getattr(self, "blendshape_nodes", {}).items():
            for idx in by_alias.values():
                dests.append(("%s.weight[%d]" % (bs, idx), {"blendshape": bs, "index": idx}))
        for c, (cname, aliases) in getattr(self, "constraint_names", {}).items():
            for i, a in enumerate(aliases):
                dests.append(("%s.%s" % (c, a), {"constraint": cname, "index": i}))
        out = []
        for plug, target in dests:
            try:
                src = _inputs(plug)
            except (RuntimeError, ValueError):
                continue
            if not src:
                continue
            src_type = cmds.nodeType(_node_of(src[0]))
            if "Constraint" in src_type or src_type in ("ikEffector", "pairBlend") or src_type.startswith("animCurveT"):
                if src_type == "pairBlend":
                    self.warn("%s: pairBlend (keys + constraint) is not converted" % plug)
                continue
            expr = self._trace(src[0], 0)
            out.append({"target": target, "expr": expr})
        return out

    def _const_or_trace(self, plug, depth, factor=1.0):
        src = _inputs(plug)
        if src:
            return self._trace(src[0], depth + 1)
        return {"op": "const", "value": cmds.getAttr(plug) * factor}

    def _trace(self, plug, depth):
        if depth > 32:
            return {"op": "unsupported", "node_type": "graph too deep"}
        node, attr = _node_of(plug), _attr_of(plug)
        ntype = cmds.nodeType(node)
        if self.name(node) is not None:
            return {"op": "plug", "node": self.name(node), "attr": attr.split(".")[-1]}
        if ntype == "blendShape":
            by_alias = getattr(self, "blendshape_nodes", {}).get(node, {})
            m = re.match(r"weight\[(\d+)\]", attr)
            idx = int(m.group(1)) if m else by_alias.get(attr)
            return {"op": "plug", "blendshape": node, "index": idx}
        if ntype == "unitConversion":
            return {"op": "affine", "scale": cmds.getAttr(node + ".conversionFactor"), "offset": 0.0,
                    "input": self._const_or_trace(node + ".input", depth)}
        if ntype.startswith("animCurveU"):
            return self._curve(node, depth)
        if ntype == "blendWeighted":
            idxs = cmds.getAttr(node + ".input", multiIndices=True) or []
            return {"op": "sum",
                    "inputs": [self._const_or_trace("%s.input[%d]" % (node, i), depth) for i in idxs],
                    "weights": [cmds.getAttr("%s.weight[%d]" % (node, i)) if
                                cmds.getAttr(node + ".weight", multiIndices=True) and
                                i in cmds.getAttr(node + ".weight", multiIndices=True) else 1.0 for i in idxs]}
        if ntype == "reverse":
            axis = attr[-1] if attr[-1] in "XYZ" else "X"
            return {"op": "affine", "scale": -1.0, "offset": 1.0,
                    "input": self._const_or_trace("%s.input%s" % (node, axis), depth)}
        if ntype in ("multDoubleLinear", "addDoubleLinear"):
            a, b = node + ".input1", node + ".input2"
            if ntype == "addDoubleLinear":
                return {"op": "sum", "inputs": [self._const_or_trace(a, depth), self._const_or_trace(b, depth)],
                        "weights": [1.0, 1.0]}
            return self._product(self._const_or_trace(a, depth), self._const_or_trace(b, depth), ntype)
        if ntype == "multiplyDivide":
            axis = attr[-1] if attr[-1] in "XYZ" else "X"
            op = cmds.getAttr(node + ".operation")
            a = self._const_or_trace("%s.input1%s" % (node, axis), depth)
            b = self._const_or_trace("%s.input2%s" % (node, axis), depth)
            if op == 0:
                return a
            if op == 1:
                return self._product(a, b, ntype)
            if op == 2 and b["op"] == "const" and b["value"] != 0.0:
                return {"op": "affine", "scale": 1.0 / b["value"], "offset": 0.0, "input": a}
            return {"op": "unsupported", "node_type": "multiplyDivide(operation=%d)" % op}
        if ntype == "plusMinusAverage":
            return self._pma(node, attr, depth)
        return {"op": "unsupported", "node_type": ntype}

    @staticmethod
    def _product(a, b, ntype):
        if b["op"] == "const":
            return {"op": "affine", "scale": b["value"], "offset": 0.0, "input": a}
        if a["op"] == "const":
            return {"op": "affine", "scale": a["value"], "offset": 0.0, "input": b}
        return {"op": "unsupported", "node_type": ntype + " of two inputs"}

    def _pma(self, node, attr, depth):
        op = cmds.getAttr(node + ".operation")
        if attr.startswith("output3D"):
            axis = attr[-1].lower()
            idxs = cmds.getAttr(node + ".input3D", multiIndices=True) or []
            plugs = ["%s.input3D[%d].input3D%s" % (node, i, axis) for i in idxs]
        else:
            idxs = cmds.getAttr(node + ".input1D", multiIndices=True) or []
            plugs = ["%s.input1D[%d]" % (node, i) for i in idxs]
        if not plugs:
            return {"op": "const", "value": 0.0}
        if op == 1:
            weights = [1.0] * len(plugs)
        elif op == 2:
            weights = [1.0] + [-1.0] * (len(plugs) - 1)
        elif op == 3:
            weights = [1.0 / len(plugs)] * len(plugs)
        else:
            return {"op": "unsupported", "node_type": "plusMinusAverage(operation=%d)" % op}
        return {"op": "sum", "inputs": [self._const_or_trace(p, depth) for p in plugs], "weights": weights}

    def _curve(self, curve, depth):
        ntype = cmds.nodeType(curve)
        factor = {"animCurveUA": self.ang, "animCurveUL": self.lin}.get(ntype, 1.0)
        xs = cmds.keyframe(curve, q=True, floatChange=True) or []
        ys = cmds.keyframe(curve, q=True, valueChange=True) or []
        itt = cmds.keyTangent(curve, q=True, inTangentType=True) or ["auto"] * len(xs)
        ott = cmds.keyTangent(curve, q=True, outTangentType=True) or ["auto"] * len(xs)
        pre = cmds.setInfinity(curve, q=True, preInfinite=True) or ["constant"]
        post = cmds.setInfinity(curve, q=True, postInfinite=True) or ["constant"]
        src = _inputs(curve + ".input")
        if src:
            inp = self._trace(src[0], depth + 1)
        else:
            inp = {"op": "unsupported", "node_type": "animCurve without driver"}
        return {"op": "curve", "input": inp, "pre": pre[0], "post": post[0],
                "keys": [[x, y * factor, i, o] for x, y, i, o in zip(xs, ys, itt, ott)]}


# --- public API ---------------------------------------------------------------

def export_rig(path, roots=None, **options):
    """Export the rig under ``roots`` (default: current selection) to ``path``."""
    roots = roots or cmds.ls(selection=True, long=True, type="transform")
    if not roots:
        raise RuntimeError("Select the rig's top group(s) (skeleton, controls and meshes) or pass roots=")
    data = RigExporter(roots, **options).run()
    with open(path, "w") as fh:
        json.dump(data, fh, separators=(",", ":"))
    for w in data["warnings"]:
        print("maya_rig_exporter warning: " + w)
    print("maya_rig_exporter: wrote %s (%d nodes, %d meshes, %d constraints, %d drivers, %d warnings)" % (
        path, len(data["nodes"]), len(data["meshes"]), len(data["constraints"]),
        len(data["drivers"]), len(data["warnings"])))
    return data


def show_ui():
    win = "m2bRigExporter"
    if cmds.window(win, exists=True):
        cmds.deleteUI(win)
    cmds.window(win, title="Maya -> Blender Rig Exporter", widthHeight=(420, 170))
    cmds.columnLayout(adjustableColumn=True, rowSpacing=6, columnAttach=("both", 8))
    cmds.text(label="Select the rig root group(s), then export.", align="left")
    path_field = cmds.textFieldButtonGrp(label="File", buttonLabel="...", columnWidth3=(40, 300, 40),
                                         adjustableColumn=2)

    def browse(*_):
        picked = cmds.fileDialog2(fileFilter="Maya2Blender rig (*.m2b.json)", dialogStyle=2, fileMode=0)
        if picked:
            cmds.textFieldButtonGrp(path_field, e=True, text=picked[0])

    cmds.textFieldButtonGrp(path_field, e=True, buttonCommand=browse)
    ns_box = cmds.checkBox(label="Strip namespaces", value=True)
    mesh_box = cmds.checkBox(label="Include meshes (skin weights, blend shapes)", value=True)

    def run(*_):
        path = cmds.textFieldButtonGrp(path_field, q=True, text=True)
        if not path:
            cmds.warning("Choose an output file first")
            return
        data = export_rig(path, strip_namespaces=cmds.checkBox(ns_box, q=True, value=True),
                          include_meshes=cmds.checkBox(mesh_box, q=True, value=True))
        cmds.confirmDialog(title="Export finished",
                           message="%d nodes, %d meshes, %d warnings\n(see Script Editor)"
                                   % (len(data["nodes"]), len(data["meshes"]), len(data["warnings"])))

    cmds.button(label="Export", command=run, height=32)
    cmds.showWindow(win)


def _main(argv):
    import argparse

    parser = argparse.ArgumentParser(description="Export a Maya rig for Blender")
    parser.add_argument("scene")
    parser.add_argument("output")
    parser.add_argument("--root", action="append", required=True, help="rig root node (repeatable)")
    parser.add_argument("--keep-namespaces", action="store_true")
    args = parser.parse_args(argv)
    import maya.standalone
    maya.standalone.initialize(name="python")
    global om, oma, cmds
    import maya.api.OpenMaya as om
    import maya.api.OpenMayaAnim as oma
    from maya import cmds
    cmds.file(args.scene, open=True, force=True)
    export_rig(args.output, roots=args.root, strip_namespaces=not args.keep_namespaces)
    maya.standalone.uninitialize()


if __name__ == "__main__":
    _main(sys.argv[1:])
