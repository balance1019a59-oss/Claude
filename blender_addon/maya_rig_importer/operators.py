import json
import os

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, StringProperty
from bpy_extras.io_utils import ImportHelper

from .core.schema import FormatError

AXIS_ITEMS = [
    ("AUTO", "Auto", "Point each joint's bone at its child (per joint)"),
    ("X", "+X", "Joint +X becomes the bone direction"),
    ("-X", "-X", "Joint -X becomes the bone direction"),
    ("Y", "+Y", "Joint +Y becomes the bone direction"),
    ("-Y", "-Y", "Joint -Y becomes the bone direction"),
    ("Z", "+Z", "Joint +Z becomes the bone direction"),
    ("-Z", "-Z", "Joint -Z becomes the bone direction"),
]


class IMPORT_OT_maya_rig(bpy.types.Operator, ImportHelper):
    """Import a rig exported from Maya with maya_rig_exporter.py"""

    bl_idname = "import_scene.maya_rig"
    bl_label = "Import Maya Rig"
    bl_options = {"REGISTER", "UNDO", "PRESET"}

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={"HIDDEN"})

    scale: FloatProperty(
        name="Scale", default=0.01, min=1e-6, soft_max=100.0, precision=4,
        description="Blender units per Maya centimetre (0.01 = cm to metres)")
    primary_axis: EnumProperty(
        name="Bone Axis", items=AXIS_ITEMS, default="AUTO",
        description="Which local joint axis becomes the Blender bone's Y axis")
    import_meshes: BoolProperty(name="Meshes", default=True, description="Import skinned and rigid meshes")
    import_constraints: BoolProperty(name="Constraints & IK", default=True)
    import_drivers: BoolProperty(name="Drivers (SDK / connections)", default=True)
    import_widgets: BoolProperty(name="Control Shapes", default=True,
                                 description="Turn NURBS control curves into bone custom shapes")

    def execute(self, context):
        from .builder import import_rig

        try:
            with open(self.filepath, encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError) as exc:
            self.report({"ERROR"}, "Cannot read %s: %s" % (self.filepath, exc))
            return {"CANCELLED"}
        if context.object is not None and context.object.mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
        name = data.get("name") or os.path.basename(self.filepath).split(".")[0]
        try:
            result = import_rig(context, data, scale=self.scale, primary_axis=self.primary_axis, name=name,
                                meshes=self.import_meshes, constraints=self.import_constraints,
                                drivers=self.import_drivers, widgets=self.import_widgets)
        except (FormatError, ValueError) as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        for w in result.warnings[:20]:
            self.report({"WARNING"}, w)
        self.report({"INFO"}, "Imported %s: %d bones, %d warnings (see text block %s_import_log)"
                    % (result.rig.name, len(result.plan.bones), len(result.warnings), result.plan.name))
        return {"FINISHED"}

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.prop(self, "scale")
        layout.prop(self, "primary_axis")
        col = layout.column(heading="Include")
        col.prop(self, "import_meshes")
        col.prop(self, "import_constraints")
        col.prop(self, "import_drivers")
        col.prop(self, "import_widgets")


def menu_func_import(self, context):
    self.layout.operator(IMPORT_OT_maya_rig.bl_idname, text="Maya Rig (.m2b.json)")


def register():
    bpy.utils.register_class(IMPORT_OT_maya_rig)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)


def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    bpy.utils.unregister_class(IMPORT_OT_maya_rig)
