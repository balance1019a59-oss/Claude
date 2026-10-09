"""Interchange-file identification and validation (see docs/FORMAT.md)."""

FORMAT_NAME = "maya2blender-rig"
FORMAT_VERSION = 1
FILE_EXTENSION = ".m2b.json"

NODE_TYPES = ("joint", "transform", "ikHandle")
CONSTRAINT_TYPES = ("parent", "point", "orient", "scale", "aim", "poleVector")


class FormatError(ValueError):
    pass


def validate(data):
    """Check structure and cross references.

    Raises :class:`FormatError` for anything that makes the file unusable and
    returns a list of warnings for problems that only drop single items.
    """
    if not isinstance(data, dict) or data.get("format") != FORMAT_NAME:
        raise FormatError("not a %s file" % FORMAT_NAME)
    version = data.get("version")
    if not isinstance(version, int) or version > FORMAT_VERSION:
        raise FormatError("unsupported format version %r (this importer reads up to %d)"
                          % (version, FORMAT_VERSION))
    units = data.get("units", {})
    if units.get("linear", "cm") != "cm" or units.get("angle", "rad") != "rad":
        raise FormatError("file must store centimetres and radians, got %r" % (units,))

    warnings = []
    names = set()
    for i, node in enumerate(data.get("nodes", [])):
        name = node.get("name")
        if not name:
            raise FormatError("node #%d has no name" % i)
        if name in names:
            raise FormatError("duplicate node name %r" % name)
        names.add(name)
        if node.get("type") not in NODE_TYPES:
            raise FormatError("node %r has unknown type %r" % (name, node.get("type")))
        m = node.get("world_matrix")
        if not isinstance(m, list) or len(m) != 16:
            raise FormatError("node %r needs a 16 element world_matrix" % name)
    for node in data.get("nodes", []):
        parent = node.get("parent")
        if parent is not None and parent not in names:
            warnings.append("node %r: parent %r is not in the file, treated as root" % (node["name"], parent))
            node["parent"] = None

    for mesh in data.get("meshes", []):
        nverts = len(mesh.get("vertices", []))
        for face in mesh.get("faces", []):
            if any(not (0 <= v < nverts) for v in face):
                raise FormatError("mesh %r has a face index out of range" % mesh.get("name"))
        skin = mesh.get("skin")
        if skin and len(skin.get("weights", [])) != nverts:
            raise FormatError("mesh %r: skin weights do not match vertex count" % mesh.get("name"))

    kept = []
    for con in data.get("constraints", []):
        if con.get("type") not in CONSTRAINT_TYPES:
            warnings.append("constraint %r: unsupported type %r skipped" % (con.get("name"), con.get("type")))
            continue
        kept.append(con)
    data["constraints"] = kept
    return warnings
