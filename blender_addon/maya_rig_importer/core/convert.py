"""Maya -> Blender space conversion and per-bone channel mapping.

Conventions
-----------
* The interchange file stores Maya *internal* units: centimetres and radians.
* Blender world = ``f * C @ maya_world`` where ``C`` turns Maya's Y-up into
  Blender's Z-up and ``f`` is the linear scale (0.01 turns cm into metres).
  Only the left side is converted, so every node keeps its Maya local axes.
* A Maya joint has no length, a Blender bone points along its local +Y.  The
  per-bone *axis matrix* ``P`` (columns = bone X/Y/Z expressed in the Maya
  node's local axes) re-labels the axes, e.g. ``"X"`` makes the joint's +X
  the bone's +Y.  Every channel mapping below goes through ``P``.
"""

from . import mathx as mx

# Maya Y-up -> Blender Z-up: (x, y, z) -> (x, -z, y)
_Y_UP_TO_Z_UP = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]

# Columns are the bone X, Y, Z axes written in the Maya node's local axes.
_AXIS_TABLE = {
    "Y": ((1, 0, 0), (0, 1, 0), (0, 0, 1)),
    "-Y": ((-1, 0, 0), (0, -1, 0), (0, 0, 1)),
    "X": ((0, 1, 0), (1, 0, 0), (0, 0, -1)),   # joint +Y stays "up" as bone +X
    "-X": ((0, 1, 0), (-1, 0, 0), (0, 0, 1)),
    "Z": ((1, 0, 0), (0, 0, 1), (0, -1, 0)),
    "-Z": ((1, 0, 0), (0, 0, -1), (0, 1, 0)),
}
PRIMARY_AXES = tuple(_AXIS_TABLE)

ATTR_CHANNELS = {
    "translateX": ("translate", 0), "translateY": ("translate", 1), "translateZ": ("translate", 2),
    "rotateX": ("rotate", 0), "rotateY": ("rotate", 1), "rotateZ": ("rotate", 2),
    "scaleX": ("scale", 0), "scaleY": ("scale", 1), "scaleZ": ("scale", 2),
}

BLENDER_PATHS = {"translate": "location", "rotate": "rotation_euler", "scale": "scale"}
TRANSFORM_TYPES = {
    "translate": ("LOC_X", "LOC_Y", "LOC_Z"),
    "rotate": ("ROT_X", "ROT_Y", "ROT_Z"),
    "scale": ("SCALE_X", "SCALE_Y", "SCALE_Z"),
}

LINEAR_UNITS_TO_CM = {
    "mm": 0.1, "millimeter": 0.1, "cm": 1.0, "centimeter": 1.0, "m": 100.0, "meter": 100.0,
    "km": 100000.0, "in": 2.54, "inch": 2.54, "ft": 30.48, "foot": 30.48, "yd": 91.44, "yard": 91.44,
}


def axis_matrix(primary):
    cols = _AXIS_TABLE[primary]
    return [[float(cols[c][r]) for c in range(3)] for r in range(3)]


def dominant_axis(v, tol=0.999):
    """Return ``(index, sign, exact)`` for the component of ``v`` with the
    largest magnitude.  ``exact`` is False when ``v`` is not axis aligned."""
    n = mx.length(v)
    if n < mx.EPS:
        return 0, 1.0, False
    k = max(range(3), key=lambda i: abs(v[i]))
    return k, (1.0 if v[k] >= 0.0 else -1.0), abs(v[k]) / n >= tol


class SpaceConverter:
    def __init__(self, up_axis="y", scale=0.01):
        self.up_axis = (up_axis or "y").lower()
        self.C = _Y_UP_TO_Z_UP if self.up_axis == "y" else mx.identity(3)
        self.f = float(scale)

    def point(self, p):
        return mx.scale(mx.mat_vec(self.C, p), self.f)

    def delta(self, d):
        return self.point(d)

    def direction(self, v):
        return mx.mat_vec(self.C, v)

    def world(self, m16):
        """Maya world matrix -> ``(rotation3, position, scales, flipped)`` in
        Blender space.  ``rotation3`` is orthonormal and keeps the node's
        Maya local axes."""
        m = mx.from_maya(m16)
        rot, scales, flipped = mx.orthonormalize(mx.matmul(self.C, mx.to_3x3(m)))
        return rot, self.point(mx.translation_of(m)), scales, flipped


class ChannelMap:
    """Maps one Maya transform channel to one Blender pose channel.

    ``blender = a * maya + b`` for the pose value (location in metres relative
    to rest, euler radians relative to rest, scale as a ratio to rest).
    """

    __slots__ = ("group", "maya_index", "index", "a", "b", "exact")

    def __init__(self, group, maya_index, index, a, b, exact):
        self.group, self.maya_index, self.index = group, maya_index, index
        self.a, self.b, self.exact = a, b, exact

    @property
    def data_path(self):
        return BLENDER_PATHS[self.group]

    @property
    def transform_type(self):
        return TRANSFORM_TYPES[self.group][self.index]

    def to_blender(self, value):
        return self.a * value + self.b

    def to_maya(self, value):
        return (value - self.b) / self.a


def build_channel_maps(node, bone_rot, parent_rot, P, f):
    """Compute the nine :class:`ChannelMap` objects for one node.

    ``bone_rot`` / ``parent_rot`` are orthonormal 3x3 rotations in Blender
    space (bone rest rotation and the Maya *DAG parent* world rotation).
    """
    jo = mx.euler_to_mat3(node.get("joint_orient", [0.0, 0.0, 0.0]), "xyz")
    bone_t = mx.transpose(bone_rot)
    a_loc = mx.matmul(bone_t, parent_rot)            # parent-space delta -> bone axes
    a_rot = mx.matmul(bone_t, mx.matmul(parent_rot, jo))  # pre-rotation frame -> bone axes
    a_scl = mx.transpose(P)                            # node local -> bone axes

    t0 = node.get("translate", [0.0, 0.0, 0.0])
    r0 = node.get("rotate", [0.0, 0.0, 0.0])
    s0 = node.get("scale", [1.0, 1.0, 1.0])
    maps = {}
    for i, axis in enumerate("XYZ"):
        k, s, exact = dominant_axis(mx.column(a_loc, i))
        maps["translate" + axis] = ChannelMap("translate", i, k, s * f, -s * f * t0[i], exact)
        k, s, exact = dominant_axis(mx.column(a_rot, i))
        maps["rotate" + axis] = ChannelMap("rotate", i, k, s, -s * r0[i], exact)
        k, _s, exact = dominant_axis(mx.column(a_scl, i))
        rest = s0[i] if abs(s0[i]) > mx.EPS else 1.0
        maps["scale" + axis] = ChannelMap("scale", i, k, 1.0 / rest, 0.0, exact)
    return maps


def blender_rotation_mode(maya_order, maps):
    """Translate a Maya ``rotateOrder`` into the Blender Euler mode that
    applies the same physical axes in the same sequence."""
    letters = []
    for axis in maya_order.lower():
        cm = maps["rotate" + axis.upper()]
        if not cm.exact:
            return "XYZ"
        letters.append("XYZ"[cm.index])
    mode = "".join(letters)
    return mode if len(set(mode)) == 3 else "XYZ"


def ik_pole_angle(base_head, base_tail, base_x_axis, ik_tail, pole_pos):
    """Blender IK ``pole_angle`` that keeps the chain in its rest pose.

    Same construction Blender uses internally: the angle between the base
    bone's X axis and the pole direction projected onto the plane
    perpendicular to the base bone.
    """
    pole_normal = mx.cross(mx.sub(ik_tail, base_head), mx.sub(pole_pos, base_head))
    bone_dir = mx.sub(base_tail, base_head)
    projected = mx.cross(pole_normal, bone_dir)
    angle = mx.angle_between(base_x_axis, projected)
    if mx.angle_between(mx.cross(base_x_axis, projected), bone_dir) < 1.0:
        angle = -angle
    return angle

