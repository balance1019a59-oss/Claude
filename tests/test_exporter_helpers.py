import maya_rig_exporter as exp


def test_expand_components():
    assert exp.expand_components(["vtx[0:2]", "vtx[5]", "garbage"]) == [0, 1, 2, 5]


def test_clean_enum_names():
    assert exp.clean_enum_names("Off:On=3:Auto") == ["Off", "On", "Auto"]


def test_round_floats_recurses_and_kills_negative_zero():
    out = exp.round_floats({"a": [1.23456789, -0.0000001], "b": (2.0,)}, 4)
    assert out == {"a": [1.2346, 0.0], "b": [2.0]}
    assert str(out["a"][1]) == "0.0"


def test_strip_namespace():
    assert exp.strip_namespace("|grp|char:L_arm") == "L_arm"


def test_format_identifiers_match_importer():
    from maya_rig_importer.core import schema
    assert exp.FORMAT_NAME == schema.FORMAT_NAME
    assert exp.FORMAT_VERSION == schema.FORMAT_VERSION
