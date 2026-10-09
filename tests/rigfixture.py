"""Builds interchange data equivalent to what maya_rig_exporter.py writes for
a small arm rig, with self-consistent Maya matrices (Y-up, cm, radians).

Rig contents:
  rig (group)
    shoulder -> elbow -> wrist -> finger1 -> finger_end   joints, X down the chain
    look_jnt                                              joint aimed at look_ctrl
    arm_ik_grp -> arm_ik_ctrl -> arm_ikh                  IK control + ikHandle
    arm_pole_ctrl                                         poleVector target
    settings_ctrl  (ikFk, curl, space custom attributes)
    prop_grp       parentConstraint [wrist, rig]
    elbow_loc      pointConstraint elbow (with offset)
    look_ctrl      aim target
  arm_geo   skinned tube with a "bulge" blend shape
"""

import math

M = math


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _t(a):
    return [list(r) for r in zip(*a)]


def _mv(a, v):
    return [sum(a[i][j] * v[j] for j in range(3)) for i in range(3)]


def _norm(v):
    n = M.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


I3 = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]


def aim_rot(x_dir, up=(0.0, 1.0, 0.0)):
    """Rotation whose X axis points along x_dir and Y is as close to up as possible."""
    x = _norm(x_dir)
    z = _norm(_cross(x, list(up)))
    y = _cross(z, x)
    return [[x[i], y[i], z[i]] for i in range(3)]


def to_euler_xyz(r):
    """Inverse of Rz @ Ry @ Rx."""
    y = M.asin(max(-1.0, min(1.0, -r[2][0])))
    x = M.atan2(r[2][1], r[2][2])
    z = M.atan2(r[1][0], r[0][0])
    return [x, y, z]


def maya_matrix(rot, pos):
    """Column rotation + position -> flat Maya (row-vector) matrix."""
    rows = [[rot[0][c], rot[1][c], rot[2][c], 0.0] for c in range(3)]
    rows.append([pos[0], pos[1], pos[2], 1.0])
    return [x for row in rows for x in row]


class _Builder:
    def __init__(self):
        self.nodes = []
        self.world = {}

    def add(self, name, ntype, parent, rot, pos, **extra):
        prot, ppos = self.world[parent] if parent else (I3, [0.0, 0.0, 0.0])
        local_rot = _mm(_t(prot), rot)
        local_pos = _mv(_t(prot), [p - q for p, q in zip(pos, ppos)])
        rec = {
            "name": name, "type": ntype, "parent": parent,
            "world_matrix": maya_matrix(rot, pos), "parent_matrix": maya_matrix(prot, ppos),
            "translate": local_pos, "rotate": [0.0, 0.0, 0.0], "scale": [1.0, 1.0, 1.0],
            "rotate_order": "xyz", "rotate_axis": [0.0, 0.0, 0.0],
            "inherits_transform": True, "visibility": True,
            "locked": {"translate": [False] * 3, "rotate": [False] * 3, "scale": [False] * 3},
        }
        if ntype == "joint":
            rec["joint_orient"] = to_euler_xyz(local_rot)
            rec["segment_scale_compensate"] = True
            rec["radius"] = 1.0
        else:
            rec["rotate"] = to_euler_xyz(local_rot)
        rec.update(extra)
        self.nodes.append(rec)
        self.world[name] = (rot, list(pos))
        return rec


def circle(center, radius=6.0, normal_axis=0, n=8):
    pts = []
    for i in range(n):
        a = 2 * M.pi * i / n
        p = list(center)
        u, v = [k for k in range(3) if k != normal_axis]
        p[u] += radius * M.cos(a)
        p[v] += radius * M.sin(a)
        pts.append(p)
    return [{"points": pts, "closed": True, "color": None}]


SHOULDER = [0.0, 150.0, 0.0]
ELBOW = [30.0, 150.0, -4.0]
WRIST = [60.0, 150.0, 0.0]
FINGER1 = [70.0, 150.0, 0.0]
FINGER_END = [80.0, 150.0, 0.0]
POLE = [30.0, 150.0, -50.0]
LOOK = [0.0, 170.0, 10.0]
LOOK_TARGET = [0.0, 170.0, 60.0]


def arm_rig():
    b = _Builder()
    b.add("rig", "transform", None, I3, [0.0, 0.0, 0.0])
    b.add("shoulder", "joint", "rig", aim_rot([e - s for e, s in zip(ELBOW, SHOULDER)]), SHOULDER)
    b.add("elbow", "joint", "shoulder", aim_rot([e - s for e, s in zip(WRIST, ELBOW)]), ELBOW)
    b.add("wrist", "joint", "elbow", aim_rot([1.0, 0.0, 0.0]), WRIST)
    b.add("finger1", "joint", "wrist", aim_rot([1.0, 0.0, 0.0]), FINGER1,
          limits={"rotate": [None, None, [0.0, M.radians(100.0)]]})
    b.add("finger_end", "joint", "finger1", aim_rot([1.0, 0.0, 0.0]), FINGER_END)
    b.add("look_jnt", "joint", "rig", aim_rot([e - s for e, s in zip(LOOK_TARGET, LOOK)]), LOOK)

    b.add("arm_ik_grp", "transform", "rig", I3, WRIST)
    b.add("arm_ik_ctrl", "transform", "arm_ik_grp", I3, WRIST, shapes=circle(WRIST), color=[0.9, 0.1, 0.1])
    b.add("arm_ikh", "ikHandle", "arm_ik_ctrl", I3, WRIST)
    b.add("arm_pole_ctrl", "transform", "rig", I3, POLE, shapes=circle(POLE, 3.0, 2), color=[0.9, 0.9, 0.1])
    b.add("settings_ctrl", "transform", "rig", I3, [0.0, 190.0, 0.0], shapes=circle([0.0, 190.0, 0.0], 3.0, 2),
          attributes=[
              {"name": "ikFk", "type": "float", "value": 1.0, "min": 0.0, "max": 1.0},
              {"name": "curl", "type": "float", "value": 0.0, "min": 0.0, "max": 10.0},
              {"name": "space", "type": "enum", "value": 0, "enum": ["wrist", "world"]},
          ],
          locked={"translate": [True] * 3, "rotate": [True] * 3, "scale": [True] * 3})
    b.add("prop_grp", "transform", "rig", I3, [65.0, 150.0, 10.0])
    b.add("elbow_loc", "transform", "rig", I3, [30.0, 155.0, -4.0])
    b.add("look_ctrl", "transform", "rig", I3, LOOK_TARGET, shapes=circle(LOOK_TARGET, 2.0, 2))

    joints = ["shoulder", "elbow", "wrist", "finger1"]
    xs = [0.0, 15.0, 30.0, 45.0, 60.0, 70.0, 80.0]
    verts, faces, weights = [], [], []
    for x in xs:
        for dy, dz in ((3, 3), (3, -3), (-3, -3), (-3, 3)):
            verts.append([x, 150.0 + dy, dz])
            if x < 15:
                w = [[0, 1.0]]
            elif x < 30:
                w = [[0, 0.5], [1, 0.5]]
            elif x < 45:
                w = [[1, 1.0]]
            elif x < 60:
                w = [[1, 0.5], [2, 0.5]]
            elif x < 70:
                w = [[2, 1.0]]
            else:
                w = [[3, 1.0]]
            weights.append(w)
    for r in range(len(xs) - 1):
        for k in range(4):
            a, c = r * 4 + k, r * 4 + (k + 1) % 4
            faces.append([a, c, c + 4, a + 4])
    mesh = {
        "name": "arm_geo", "vertices": verts, "faces": faces, "hard_edges": [],
        "uvs": {"name": "map1", "coords": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
                "face_uvs": [[0, 1, 2, 3] for _ in faces]},
        "skin": {"node": "skinCluster1", "influences": joints, "weights": weights},
        "blendshapes": [{"node": "arm_bs", "mesh": "arm_geo", "targets": [
            {"name": "bulge", "index": 0, "weight": 0.0,
             "deltas": {"indices": [8, 9, 10, 11], "offsets": [[0.0, 2.0, 0.0]] * 4}, "inbetweens": []}]}],
    }

    constraints = [
        {"name": "arm_pv", "type": "poleVector", "driven": "arm_ikh",
         "targets": [{"node": "arm_pole_ctrl", "weight": 1.0}]},
        {"name": "wrist_orient", "type": "orient", "driven": "wrist",
         "targets": [{"node": "arm_ik_ctrl", "weight": 1.0}],
         "skip": {"translate": [True] * 3, "rotate": [False] * 3, "scale": [True] * 3}},
        {"name": "prop_parent", "type": "parent", "driven": "prop_grp",
         "targets": [{"node": "wrist", "weight": 1.0}, {"node": "rig", "weight": 0.0}],
         "skip": {"translate": [False] * 3, "rotate": [False] * 3, "scale": [True] * 3}},
        {"name": "elbow_loc_point", "type": "point", "driven": "elbow_loc",
         "targets": [{"node": "elbow", "weight": 1.0}],
         "skip": {"translate": [False] * 3, "rotate": [True] * 3, "scale": [True] * 3}},
        {"name": "look_aim", "type": "aim", "driven": "look_jnt",
         "targets": [{"node": "look_ctrl", "weight": 1.0}],
         "aim_vector": [1.0, 0.0, 0.0], "up_vector": [0.0, 1.0, 0.0], "world_up_type": "scene",
         "world_up_vector": [0.0, 1.0, 0.0], "world_up_object": None, "offset": [0.0, 0.0, 0.0]},
    ]
    ik = [{"name": "arm_ikh", "solver": "rp", "start_joint": "shoulder", "end_joint": "wrist",
           "pole_vector": [0.0, 0.0, -1.0], "twist": 0.0, "ik_blend": 1.0}]
    curl_plug = {"op": "plug", "node": "settings_ctrl", "attr": "curl"}
    drivers = [
        {"target": {"node": "arm_ikh", "attr": "ikBlend"},
         "expr": {"op": "plug", "node": "settings_ctrl", "attr": "ikFk"}},
        # set driven key: curl 0..10 -> finger1.rotateZ 0..90 degrees (Maya UI 'deg' converted to rad)
        {"target": {"node": "finger1", "attr": "rotateZ"},
         "expr": {"op": "curve", "input": curl_plug, "pre": "constant", "post": "constant",
                  "keys": [[0.0, 0.0, "linear", "linear"], [10.0, M.radians(90.0), "linear", "linear"]]}},
        {"target": {"blendshape": "arm_bs", "index": 0},
         "expr": {"op": "curve", "input": curl_plug, "pre": "constant", "post": "constant",
                  "keys": [[0.0, 0.0, "linear", "linear"], [10.0, 1.0, "linear", "linear"]]}},
        # space switch: world weight = space, wrist weight = 1 - space (reverse node)
        {"target": {"constraint": "prop_parent", "index": 1},
         "expr": {"op": "plug", "node": "settings_ctrl", "attr": "space"}},
        {"target": {"constraint": "prop_parent", "index": 0},
         "expr": {"op": "affine", "scale": -1.0, "offset": 1.0,
                  "input": {"op": "plug", "node": "settings_ctrl", "attr": "space"}}},
    ]
    return {
        "format": "maya2blender-rig", "version": 1, "name": "arm_rig",
        "source": {"application": "maya", "up_axis": "y", "ui_linear_unit": "cm"},
        "units": {"linear": "cm", "angle": "rad"},
        "nodes": b.nodes, "meshes": [mesh], "constraints": constraints, "ik_handles": ik,
        "drivers": drivers, "warnings": [],
    }
