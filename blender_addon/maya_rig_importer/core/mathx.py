"""Tiny dependency-free matrix helpers.

All matrices are lists of row lists and use the *column-vector* convention
(``p' = M @ p``), which is what Blender's ``mathutils`` uses.  Maya stores
matrices in the row-vector convention, see :func:`from_maya`.
"""

import math

EPS = 1e-9


def identity(n=4):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def matmul(a, b):
    inner = len(b)
    return [[sum(a[i][t] * b[t][j] for t in range(inner)) for j in range(len(b[0]))]
            for i in range(len(a))]


def chain(*mats):
    out = mats[0]
    for m in mats[1:]:
        out = matmul(out, m)
    return out


def transpose(a):
    return [list(col) for col in zip(*a)]


def mat_vec(m, v):
    return [sum(m[i][j] * v[j] for j in range(len(v))) for i in range(len(m))]


# --- vectors --------------------------------------------------------------

def add(a, b):
    return [x + y for x, y in zip(a, b)]


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def scale(v, s):
    return [x * s for x in v]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def length(v):
    return math.sqrt(dot(v, v))


def normalized(v):
    n = length(v)
    if n < EPS:
        return [0.0] * len(v)
    return [x / n for x in v]


def angle_between(a, b):
    na, nb = length(a), length(b)
    if na < EPS or nb < EPS:
        return 0.0
    c = max(-1.0, min(1.0, dot(a, b) / (na * nb)))
    return math.acos(c)


# --- 3x3 / 4x4 ------------------------------------------------------------

def column(m, j):
    return [m[i][j] for i in range(len(m))]


def from_columns(*cols):
    return [[cols[j][i] for j in range(len(cols))] for i in range(len(cols[0]))]


def to_3x3(m4):
    return [row[:3] for row in m4[:3]]


def compose(rot3, translation):
    """Build a 4x4 from a 3x3 rotation/linear part and a translation."""
    out = [list(rot3[i]) + [translation[i]] for i in range(3)]
    out.append([0.0, 0.0, 0.0, 1.0])
    return out


def translation_of(m4):
    return [m4[0][3], m4[1][3], m4[2][3]]


def from_maya(m16):
    """Convert a flat 16-float Maya matrix (row-vector convention, translation
    in elements 12-14) into a 4x4 column-vector matrix."""
    rows = [list(m16[i * 4:i * 4 + 4]) for i in range(4)]
    return transpose(rows)


def determinant3(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def inverse(m):
    """General inverse via Gauss-Jordan elimination (any square size)."""
    n = len(m)
    aug = [list(map(float, m[i])) + [1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1e-12:
            raise ValueError("matrix is singular")
        aug[col], aug[pivot] = aug[pivot], aug[col]
        p = aug[col][col]
        aug[col] = [x / p for x in aug[col]]
        for r in range(n):
            if r != col and aug[r][col] != 0.0:
                f = aug[r][col]
                aug[r] = [x - f * y for x, y in zip(aug[r], aug[col])]
    return [row[n:] for row in aug]


def orthonormalize(m3):
    """Gram-Schmidt on the columns (X kept, then Y, Z rebuilt).

    Returns ``(rotation, scales, flipped)``; ``flipped`` is True when the input
    had a negative determinant (mirrored transform) and Z was negated to keep
    the result a proper rotation.
    """
    x, y, z = column(m3, 0), column(m3, 1), column(m3, 2)
    scales = [length(x), length(y), length(z)]
    flipped = determinant3(m3) < 0.0
    xn = normalized(x)
    if length(xn) < 0.5:
        xn = [1.0, 0.0, 0.0]
    y = sub(y, scale(xn, dot(y, xn)))
    yn = normalized(y)
    if length(yn) < 0.5:
        yn = normalized(cross(normalized(z) if length(z) > EPS else [0.0, 0.0, 1.0], xn))
        if length(yn) < 0.5:
            yn = [0.0, 1.0, 0.0] if abs(xn[1]) < 0.9 else [0.0, 0.0, 1.0]
            yn = normalized(sub(yn, scale(xn, dot(yn, xn))))
    zn = cross(xn, yn)
    return from_columns(xn, yn, zn), scales, flipped


def rot_axis(axis, angle):
    """3x3 rotation about a principal axis ('x', 'y' or 'z')."""
    c, s = math.cos(angle), math.sin(angle)
    axis = axis.lower()
    if axis == "x":
        return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
    if axis == "y":
        return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
    if axis == "z":
        return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    raise ValueError(axis)


def euler_to_mat3(angles, order="xyz"):
    """Euler (radians) to 3x3.  ``order`` lists axes in application order,
    which is the same meaning for Maya ``rotateOrder`` and Blender Euler modes
    (``"xyz"`` -> ``Rz @ Ry @ Rx``)."""
    out = identity(3)
    for axis in order.lower():
        out = matmul(rot_axis(axis, angles["xyz".index(axis)]), out)
    return out


def is_identity3(m, tol=1e-5):
    return all(abs(m[i][j] - (1.0 if i == j else 0.0)) <= tol for i in range(3) for j in range(3))
