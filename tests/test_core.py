import copy
import math

import pytest

from maya_rig_importer.core import mathx as mx
from maya_rig_importer.core.convert import (PRIMARY_AXES, SpaceConverter, axis_matrix, blender_rotation_mode,
                                            build_channel_maps, ik_pole_angle)
from maya_rig_importer.core.plan import RigPlanner, make_plan
from maya_rig_importer.core.schema import FormatError, validate
from rigfixture import arm_rig


def close(a, b, tol=1e-6):
    if isinstance(a, (list, tuple)):
        return all(close(x, y, tol) for x, y in zip(a, b)) and len(a) == len(b)
    if isinstance(a, str):
        return a == b
    return abs(a - b) <= tol


# --- math -------------------------------------------------------------------

def test_inverse_roundtrip():
    m = mx.compose(mx.euler_to_mat3([0.3, -0.7, 1.1], "zxy"), [1.0, 2.0, 3.0])
    assert close(mx.matmul(m, mx.inverse(m)), mx.identity(4))


def test_euler_order_matches_application_order():
    # "xyz": rotate about X first, so Rz @ Ry @ Rx
    a = [0.2, 0.4, 0.6]
    expected = mx.chain(mx.rot_axis("z", 0.6), mx.rot_axis("y", 0.4), mx.rot_axis("x", 0.2))
    assert close(mx.euler_to_mat3(a, "xyz"), expected)


def test_from_maya_transposes_row_vector_matrix():
    flat = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 5, 6, 7, 1]
    assert mx.translation_of(mx.from_maya(flat)) == [5, 6, 7]


def test_orthonormalize_strips_scale_and_reports_mirror():
    rot, scales, flipped = mx.orthonormalize([[2.0, 0, 0], [0, 3.0, 0], [0, 0, -1.0]])
    assert close(scales, [2.0, 3.0, 1.0]) and flipped
    assert close(mx.determinant3(rot), 1.0)


# --- conversion ---------------------------------------------------------------

def test_y_up_to_z_up():
    conv = SpaceConverter("y", 0.01)
    assert close(conv.point([0.0, 100.0, 0.0]), [0.0, 0.0, 1.0])   # up stays up
    assert close(conv.point([0.0, 0.0, 100.0]), [0.0, -1.0, 0.0])  # Maya front (+Z) -> Blender front (-Y)
    assert close(SpaceConverter("z", 1.0).point([1, 2, 3]), [1, 2, 3])


@pytest.mark.parametrize("axis", PRIMARY_AXES)
def test_axis_matrices_are_rotations_pointing_bone_y_down_primary(axis):
    P = axis_matrix(axis)
    assert close(mx.determinant3(P), 1.0)
    expected = [0.0, 0.0, 0.0]
    expected["XYZ".index(axis[-1])] = -1.0 if axis.startswith("-") else 1.0
    assert close(mx.column(P, 1), expected)


def test_channel_maps_for_x_primary_joint():
    node = {"translate": [10.0, 0.0, 0.0], "rotate": [0.0, 0.0, 0.0], "scale": [1, 1, 1],
            "joint_orient": [0.0, 0.0, 0.0]}
    P = axis_matrix("X")
    maps = build_channel_maps(node, mx.matmul(mx.identity(3), P), mx.identity(3), P, 0.01)
    rx, rz, tx = maps["rotateX"], maps["rotateZ"], maps["translateX"]
    assert (rx.index, rx.a) == (1, 1.0)            # twist about the bone
    assert (rz.index, rz.a) == (2, -1.0) and rz.exact
    assert tx.index == 1 and close(tx.to_blender(10.0), 0.0) and close(tx.to_blender(20.0), 0.1)
    assert blender_rotation_mode("xyz", maps) == "YXZ"


def test_pole_angle_zero_when_pole_along_bone_x():
    # base bone along +Y with X axis +X; pole on +X -> zero pole angle
    angle = ik_pole_angle([0, 0, 0], [0, 1, 0], [1, 0, 0], [0, 2, 0], [5, 1, 0])
    assert close(angle, 0.0)
    assert close(abs(ik_pole_angle([0, 0, 0], [0, 1, 0], [1, 0, 0], [0, 2, 0], [0, 1, 5])), math.pi / 2)


# --- schema ---------------------------------------------------------------------

def test_validate_rejects_foreign_files():
    with pytest.raises(FormatError):
        validate({"format": "something"})
    data = arm_rig()
    data["version"] = 99
    with pytest.raises(FormatError):
        validate(data)


def test_validate_drops_unknown_constraint_types():
    data = arm_rig()
    data["constraints"].append({"name": "geo", "type": "geometry"})
    warnings = validate(data)
    assert any("geo" in w for w in warnings)
    assert all(c["type"] != "geometry" for c in data["constraints"])


# --- planning ---------------------------------------------------------------------

@pytest.fixture
def plan():
    data = arm_rig()
    validate(data)
    return make_plan(data)


def test_every_node_becomes_a_bone_and_parents_come_first(plan):
    names = [b.name for b in plan.bones]
    for node in arm_rig()["nodes"]:
        assert node["name"] in names
    seen = set()
    for b in plan.bones:
        assert b.parent is None or b.parent in seen
        seen.add(b.name)


def test_joint_bones_reach_their_child(plan):
    shoulder, elbow = plan.bone("shoulder"), plan.bone("elbow")
    tail = mx.add(mx.translation_of(shoulder.matrix), mx.scale(mx.column(shoulder.matrix, 1)[:3], shoulder.length))
    assert close(tail, mx.translation_of(elbow.matrix), 1e-6)


def test_bone_classification(plan):
    assert plan.bone("shoulder").deform and plan.bone("shoulder").collection == "DEF"
    assert not plan.bone("finger_end").deform  # not a skin influence
    assert plan.bone("arm_ik_ctrl").collection == "CTRL" and plan.bone("arm_ik_ctrl").shape
    assert plan.bone("arm_ik_ctrl").color == [0.9, 0.1, 0.1]
    assert plan.bone("rig").collection == "MCH"
    assert plan.bone("settings_ctrl").lock_location == [True] * 3


def test_constraints_and_ik(plan):
    by_name = {c.name: c for c in plan.constraints}
    ik = by_name["arm_ikh"]
    assert ik.type == "IK" and ik.owner == "elbow" and ik.settings["chain_count"] == 2
    assert ik.targets[0] == {"subtarget": "arm_ikh", "pole_subtarget": "arm_pole_ctrl"}
    prop = by_name["prop_parent"]
    assert prop.type == "ARMATURE" and plan.bone("prop_grp").parent is None
    assert [t["subtarget"] for t in prop.targets] == ["wrist", "rig"]
    assert by_name["wrist_orient"].type == "COPY_ROTATION"
    assert by_name["look_aim"].type == "TRACK_TO"
    assert by_name["look_aim"].settings == {"track_axis": "TRACK_Y", "up_axis": "UP_X", "use_target_z": False}
    limit = by_name["Maya rotate limits"]
    assert limit.owner == "finger1" and close(limit.settings["min_z"], -math.radians(100))
    point = by_name["elbow_loc_point"]
    offset_helper = plan.bone(point.targets[0]["subtarget"])
    assert offset_helper.parent.startswith("MCH-elbow_loc-pt0-src")


def test_drivers(plan):
    by_label = {d.label: d for d in plan.drivers}
    sdk = by_label["finger1.rotateZ"]
    assert sdk.data_path == 'pose.bones["finger1"].rotation_euler' and sdk.index == 2
    assert sdk.variables[0].data_path == 'pose.bones["settings_ctrl"]["curl"]'
    assert close([k[1] for k in sdk.keys], [0.0, -math.pi / 2])   # Maya +Z -> bone -Z
    ik = by_label["arm_ikh.ikBlend"]
    assert ik.data_path == 'pose.bones["elbow"].constraints["arm_ikh"].influence'
    w0 = by_label["prop_parent.target[0].weight"]
    assert w0.data_path.endswith('.targets[0].weight') and close(w0.keys, [(0.0, 1.0, "LINEAR"), (1.0, 0.0, "LINEAR")])
    bs = by_label["arm_bs.weight[0]"]
    assert bs.id_type == "KEY" and bs.id_name == "arm_geo" and bs.data_path == 'key_blocks["bulge"].value'


def test_driver_compiler_handles_sums_and_reversed_curves():
    data = arm_rig()
    curl = {"op": "plug", "node": "settings_ctrl", "attr": "curl"}
    ikfk = {"op": "plug", "node": "settings_ctrl", "attr": "ikFk"}
    data["drivers"] = [
        {"target": {"node": "finger1", "attr": "rotateY"},
         "expr": {"op": "sum", "weights": [1.0, 2.0], "inputs": [
             {"op": "curve", "input": curl, "keys": [[0, 0, "linear", "linear"], [10, 1, "linear", "linear"]]},
             {"op": "curve", "input": {"op": "affine", "scale": -1.0, "offset": 1.0, "input": ikfk},
              "keys": [[0, 0, "linear", "step"], [1, 1, "linear", "linear"]]},
         ]}},
        {"target": {"node": "finger1", "attr": "rotateX"}, "expr": {"op": "unsupported", "node_type": "condition"}},
        {"target": {"node": "finger1", "attr": "scaleX"}, "expr": {"op": "const", "value": 2.0}},
    ]
    planner = RigPlanner(data)
    plan = planner.build()
    assert any("condition" in w for w in plan.warnings)
    assert len(plan.object_props) == 2   # both curve terms are materialised
    final = [d for d in plan.drivers if d.label == "finger1.rotateY" and d.data_path.startswith("pose")][0]
    assert final.driver_type == "SUM" and len(final.variables) == 2
    reversed_curve = [d for d in plan.drivers if d.data_path == '["m2b_002"]'][0]
    # x was reversed (ikFk = 1 - x); keys must be re-sorted with the step interp moved
    assert [k[0] for k in reversed_curve.keys] == [0.0, 1.0]
    assert [k[1] for k in reversed_curve.keys] == [2.0, 0.0]
    assert reversed_curve.keys[0][2] == "CONSTANT"
    assert plan.static_values == [("OBJECT", None, 'pose.bones["finger1"].scale', 1, 2.0)]


def test_stacked_influences_average_targets():
    assert RigPlanner._stacked_influences([1.0, 1.0, 1.0]) == [1.0, 0.5, pytest.approx(1 / 3)]
    assert RigPlanner._stacked_influences([1.0, 0.0]) == [1.0, 0.0]


def test_rest_matrices_are_orthonormal(plan):
    for b in plan.bones:
        r = mx.to_3x3(b.matrix)
        assert close(mx.matmul(mx.transpose(r), r), mx.identity(3), 1e-9), b.name


def test_fixed_primary_axis_option():
    data = arm_rig()
    plan = make_plan(copy.deepcopy(data), primary_axis="Y")
    shoulder = plan.bone("shoulder")
    # Maya joint +Y is world up -> Blender +Z
    assert close(mx.column(shoulder.matrix, 1)[:3], [0.0, 0.0, 1.0], 1e-6)
    with pytest.raises(ValueError):
        make_plan(data, primary_axis="W")
