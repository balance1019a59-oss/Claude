"""End-to-end import inside Blender (runs when the ``bpy`` module is installed:
``pip install bpy`` matching your Python, or run pytest from Blender's Python)."""

import math

import pytest

bpy = pytest.importorskip("bpy")
from mathutils import Vector  # noqa: E402

from maya_rig_importer.builder import import_rig  # noqa: E402
from rigfixture import LOOK, LOOK_TARGET, WRIST, arm_rig  # noqa: E402

F = 0.01


def to_bl(p):
    return Vector((p[0] * F, -p[2] * F, p[1] * F))


@pytest.fixture
def result():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    res = import_rig(bpy.context, arm_rig())
    bpy.context.view_layer.update()
    return res


def evaluate():
    # editing ID properties from Python does not tag the depsgraph (the UI does)
    for obj in bpy.data.objects:
        obj.update_tag()
    bpy.context.view_layer.update()


def max_rest_error(rig):
    worst, name = 0.0, None
    for pb in rig.pose.bones:
        diff = max(abs(a - b) for ra, rb in zip(pb.matrix, pb.bone.matrix_local) for a, b in zip(ra, rb))
        if diff > worst:
            worst, name = diff, pb.name
    return worst, name


def test_rest_pose_is_preserved_with_all_constraints_and_drivers(result):
    worst, name = max_rest_error(result.rig)
    assert worst < 1e-4, "bone %s moved by %.6f at rest" % (name, worst)


def test_structure(result):
    rig = result.rig
    assert {"DEF", "CTRL", "MCH"} <= {c.name for c in rig.data.collections}
    assert rig.pose.bones["arm_ik_ctrl"].custom_shape is not None
    mesh = result.mesh_objects["arm_geo"]
    assert {g.name for g in mesh.vertex_groups} == {"shoulder", "elbow", "wrist", "finger1"}
    assert mesh.modifiers["Armature"].object == rig
    assert [k.name for k in mesh.data.shape_keys.key_blocks] == ["Basis", "bulge"]
    assert mesh.data.uv_layers["map1"]
    assert bpy.data.texts.get("arm_rig_import_log")


def test_ik_follows_control(result):
    rig = result.rig
    ctrl = rig.pose.bones["arm_ik_ctrl"]
    # move the control 10 cm toward the shoulder (Maya -X) and 5 cm up (Maya +Y)
    ctrl.location = (-0.10, 0.05, 0.0)   # bone axes == Maya axes for controls
    evaluate()
    expected = to_bl([WRIST[0] - 10.0, WRIST[1] + 5.0, WRIST[2]])
    assert (rig.pose.bones["wrist"].head - expected).length < 1e-3
    # elbow stays on the pole side (Maya -Z -> Blender +Y)
    assert rig.pose.bones["elbow"].head.y > 0.0


def test_ik_fk_switch_drives_influence(result):
    rig = result.rig
    settings = rig.pose.bones["settings_ctrl"]
    settings["ikFk"] = 0.0
    rig.pose.bones["arm_ik_ctrl"].location = (-0.10, 0.0, 0.0)
    evaluate()
    assert rig.pose.bones["elbow"].constraints["arm_ikh"].influence == pytest.approx(0.0)
    assert (rig.pose.bones["wrist"].head - to_bl(WRIST)).length < 1e-4


def test_set_driven_key_and_blendshape_driver(result):
    rig = result.rig
    rig.pose.bones["settings_ctrl"]["curl"] = 10.0
    evaluate()
    finger = rig.pose.bones["finger1"]
    assert finger.rotation_euler[2] == pytest.approx(-math.pi / 2, abs=1e-4)
    # Maya +Z rotation of an X-down joint swings its tip toward Maya +Y (Blender +Z)
    assert finger.tail.z > finger.head.z + 0.09
    key = result.mesh_objects["arm_geo"].data.shape_keys.key_blocks["bulge"]
    assert key.value == pytest.approx(1.0)


def test_rotation_limit_converted(result):
    rig = result.rig
    finger = rig.pose.bones["finger1"]
    con = finger.constraints["Maya rotate limits"]
    assert con.use_limit_z and con.min_z == pytest.approx(-math.radians(100))


def test_parent_constraint_space_switch(result):
    rig = result.rig
    prop = rig.pose.bones["prop_grp"]
    rest = prop.head.copy()
    rig.pose.bones["arm_ik_ctrl"].location = (-0.10, 0.0, 0.0)
    evaluate()
    assert (prop.head - rest).length > 0.01   # follows the wrist
    rig.pose.bones["settings_ctrl"]["space"] = 1
    evaluate()
    con = prop.constraints["prop_parent"]
    assert con.targets[0].weight == pytest.approx(0.0) and con.targets[1].weight == pytest.approx(1.0)
    assert (prop.head - rest).length < 1e-4   # world space: stays put


def test_point_constraint_keeps_offset(result):
    rig = result.rig
    loc, elbow = rig.pose.bones["elbow_loc"], rig.pose.bones["elbow"]
    offset = loc.head - elbow.head
    rig.pose.bones["arm_ik_ctrl"].location = (-0.15, 0.05, 0.0)
    evaluate()
    assert (loc.head - elbow.head - offset).length < 1e-4
    assert loc.matrix.to_quaternion().rotation_difference(
        loc.bone.matrix_local.to_quaternion()).angle < 1e-4   # rotation untouched


def test_aim_constraint(result):
    rig = result.rig
    target = rig.pose.bones["look_ctrl"]
    target.location = (0.3, 0.0, 0.0)   # Maya +X
    evaluate()
    look = rig.pose.bones["look_jnt"]
    expected = (to_bl([LOOK_TARGET[0] + 30.0, LOOK_TARGET[1], LOOK_TARGET[2]]) - to_bl(LOOK)).normalized()
    assert (look.y_axis - expected).length < 1e-4
    assert look.x_axis.z > 0.9    # joint Y (up) stays up


def test_skinned_mesh_deforms(result):
    rig = result.rig
    mesh = result.mesh_objects["arm_geo"]
    depsgraph = bpy.context.evaluated_depsgraph_get()
    before = [v.co.copy() for v in mesh.evaluated_get(depsgraph).data.vertices]
    rig.pose.bones["arm_ik_ctrl"].location = (-0.15, 0.05, 0.0)
    evaluate()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    after = [v.co.copy() for v in mesh.evaluated_get(depsgraph).data.vertices]

    def ring_center(verts, ring):
        return sum(verts[ring * 4:ring * 4 + 4], Vector()) / 4

    assert (ring_center(before, 0) - ring_center(after, 0)).length < 1e-5   # pivots about the shoulder
    wrist = rig.pose.bones["wrist"].head
    assert (ring_center(after, 4) - wrist).length < 1e-4   # wrist ring follows the IK


def test_operator_imports_file(tmp_path):
    import json

    import maya_rig_importer

    bpy.ops.wm.read_factory_settings(use_empty=True)
    path = tmp_path / "arm.m2b.json"
    path.write_text(json.dumps(arm_rig()))
    maya_rig_importer.register()
    try:
        assert bpy.ops.import_scene.maya_rig(filepath=str(path), scale=1.0, import_meshes=False) == {"FINISHED"}
    finally:
        maya_rig_importer.unregister()
    rig = bpy.data.objects["arm_rig"]
    assert rig.type == "ARMATURE" and "arm_geo" not in bpy.data.objects
    assert rig.data.bones["wrist"].head_local.x == pytest.approx(60.0)   # scale 1: one Maya cm per unit


@pytest.mark.parametrize("axis", ["X", "-X", "Y", "-Y", "Z", "-Z"])
def test_rest_pose_for_every_bone_axis(axis):
    data = arm_rig()
    for node in data["nodes"]:
        if node["name"] == "finger1":
            node["rotate_order"] = "zxy"
    bpy.ops.wm.read_factory_settings(use_empty=True)
    res = import_rig(bpy.context, data, primary_axis=axis)
    evaluate()
    worst, name = max_rest_error(res.rig)
    assert worst < 1e-4, "bone %s moved by %.6f at rest" % (name, worst)
    # the SDK still turns the finger about Maya Z whatever the bone axis is
    res.rig.pose.bones["settings_ctrl"]["curl"] = 10.0
    evaluate()
    finger = res.rig.pose.bones["finger1"]
    tip = finger.matrix @ Vector((0.0, 0.0, 0.0))
    child = res.rig.pose.bones["finger_end"].head
    assert child.z - tip.z == pytest.approx(0.1, abs=1e-3)   # finger now points Maya +Y (Blender +Z)
