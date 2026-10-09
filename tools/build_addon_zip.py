"""Package the Blender add-on as an installable extension zip.

    python tools/build_addon_zip.py            -> dist/maya_rig_importer-<version>.zip

Install in Blender 4.2+: Edit > Preferences > Get Extensions > (v) Install from Disk.
"""

import os
import re
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "blender_addon", "maya_rig_importer")


def main():
    with open(os.path.join(SRC, "blender_manifest.toml"), encoding="utf-8") as fh:
        version = re.search(r'^version\s*=\s*"([^"]+)"', fh.read(), re.M).group(1)
    out_dir = os.path.join(ROOT, "dist")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "maya_rig_importer-%s.zip" % version)
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as zf:
        for folder, dirs, files in os.walk(SRC):
            dirs[:] = [d for d in dirs if d != "__pycache__"]
            for name in files:
                if name.endswith((".pyc", ".pyo")):
                    continue
                path = os.path.join(folder, name)
                zf.write(path, os.path.relpath(path, SRC))   # manifest at the zip root
    print(out)


if __name__ == "__main__":
    main()
