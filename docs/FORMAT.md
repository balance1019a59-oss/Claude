# `.m2b.json` interchange format (version 1)

Written by `maya_exporter/maya_rig_exporter.py`, read by the Blender add-on.
All values are in **Maya internal units: centimetres and radians**, in Maya's
world space (Y-up unless `source.up_axis` says `z`). Matrices are Maya's flat
16-float row-vector layout (translation in elements 12–14).

```jsonc
{
  "format": "maya2blender-rig", "version": 1, "name": "hero",
  "source": {"application": "maya", "up_axis": "y", "ui_linear_unit": "cm", ...},
  "units": {"linear": "cm", "angle": "rad"},
  "nodes": [ ... ], "meshes": [ ... ], "constraints": [ ... ],
  "ik_handles": [ ... ], "drivers": [ ... ], "warnings": [ "..." ]
}
```

## nodes

One record per exported transform; each one becomes a bone. Parents come
before their children.

| key | meaning |
|---|---|
| `name`, `type` | unique name; `joint`, `ikHandle` or `transform` |
| `parent` | parent node name or `null` |
| `world_matrix`, `parent_matrix` | rest-pose world matrix of the node and of its DAG parent |
| `translate`, `rotate`, `scale`, `rotate_order`, `rotate_axis` | channel values at rest |
| `joint_orient`, `segment_scale_compensate`, `radius` | joints only |
| `inherits_transform`, `visibility` | booleans |
| `locked` | `{"translate": [bx, by, bz], "rotate": [...], "scale": [...]}` (locked or hidden) |
| `limits` | `{"rotate": [[min\|null, max\|null] × 3], "translate": ..., "scale": ...}` |
| `shapes` | NURBS curve shapes: `[{"points": [[x,y,z], ...] (world), "closed": bool, "color": [r,g,b]\|null}]` |
| `color` | display override colour (0–1 RGB) |
| `attributes` | custom attributes: `{"name", "type": float\|angle\|linear\|int\|bool\|enum, "value", "min", "max", "enum": [...]}` |

## meshes

`name`, `vertices` (world), `faces` (vertex index lists), `hard_edges`
(`[v0, v1]` pairs), `uvs` (`{"name", "coords": [[u,v]], "face_uvs": [[uv index per corner]]}`),
`skin` (`{"influences": [node], "weights": [[[influence index, weight], ...] per vertex]}`),
`parent` (for rigid, non-skinned meshes) and `blendshapes`
(`[{"node", "targets": [{"name", "index", "weight", "deltas": {"indices": [...], "offsets": [[dx,dy,dz]] (world)}}]}]`).

## constraints

`{"name", "type": parent|point|orient|scale|aim|poleVector, "driven", "targets": [{"node", "weight"}], "skip": {...}}`.
Aim constraints add `aim_vector`, `up_vector`, `world_up_type`
(`scene|object|objectrotation|vector|none`), `world_up_vector`, `world_up_object`, `offset`.

## ik_handles

`{"name", "solver": rp|sc|spline, "start_joint", "end_joint", "pole_vector", "twist", "ik_blend"}`.
The handle itself is also a node (so it can be parented or constrained); a pole
target is a `poleVector` constraint whose `driven` is the handle.

## drivers

`{"target": <plug>, "expr": <expression>}` — one per driven attribute.

Plugs: `{"node", "attr"}` (transform channel, `visibility`, `ikBlend` or a custom
attribute), `{"blendshape", "index"}`, `{"constraint", "index"}` (target weight).

Expressions (values in internal units):

| op | fields | meaning |
|---|---|---|
| `plug` | as above | read an attribute |
| `const` | `value` | constant |
| `affine` | `scale`, `offset`, `input` | `scale * input + offset` (unitConversion, reverse, multiply by constant…) |
| `curve` | `input`, `keys: [[x, y, inTangent, outTangent]]`, `pre`, `post` | set driven key (animCurveU*) |
| `sum` | `inputs`, `weights` | blendWeighted, plusMinusAverage, addDoubleLinear |
| `unsupported` | `node_type` | anything else; the importer skips it with a warning |
