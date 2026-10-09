"""Maya Rig Importer: rebuilds a Maya rig (exported with maya_rig_exporter.py)
as a Blender armature with constraints, IK, drivers, control shapes and
skinned meshes."""

bl_info = {
    "name": "Maya Rig Importer",
    "author": "maya2blender contributors",
    "version": (0, 1, 0),
    "blender": (4, 2, 0),
    "location": "File > Import > Maya Rig (.m2b.json)",
    "description": "Convert Maya rigs (joints, controls, constraints, IK, set driven keys, skin) to Blender",
    "category": "Import-Export",
}


def register():
    from . import operators
    operators.register()


def unregister():
    from . import operators
    operators.unregister()
