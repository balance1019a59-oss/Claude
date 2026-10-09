# Maya → Blender Rig Converter

把 Maya 的 rig（骨架、控制器、約束、IK、Set Driven Key、蒙皮、BlendShape）轉成可以直接在 Blender 裡操作的 Armature。

FBX 只能帶過骨架和蒙皮，控制器、約束、IK、SDK 都會遺失。這個工具分兩步：

1. **Maya 端**：`maya_exporter/maya_rig_exporter.py` 把 rig 匯出成一個 JSON 檔（`.m2b.json`）。
2. **Blender 端**：`blender_addon/maya_rig_importer` 外掛讀進來，重建成一個 Armature 物件。

## 會轉換的內容

| Maya | Blender |
|---|---|
| joint / 控制器 / 群組 / ikHandle（每個 transform） | 同一個 Armature 裡的一根骨頭（保留階層） |
| joint 的 jointOrient、rotateOrder | 骨頭方向（預設自動讓骨頭指向子關節）、Euler 旋轉順序 |
| segmentScaleCompensate | `inherit_scale = NONE` |
| NURBS 曲線控制器、顏色 | 骨頭 Custom Shape、骨頭顏色 |
| 鎖定 / 隱藏的通道、transform limits、joint limits | Lock 開關、Limit Rotation / Location / Scale 約束 |
| 自訂屬性（float / int / bool / enum / angle / distance） | Pose bone 自訂屬性（含 min / max） |
| parentConstraint（含多目標、space switch） | Armature 約束（權重可被 driver 驅動） |
| pointConstraint / orientConstraint / scaleConstraint | Copy Location / Rotation / Scale ＋ `MCH-` 輔助骨（保留 maintain offset） |
| aimConstraint | Track To / Damped Track（＋ Locked Track 處理 object up） |
| ikHandle（RP / SC）＋ poleVectorConstraint | IK 約束，pole angle 依 rest pose 自動計算；ikBlend → influence |
| Set Driven Key、blendWeighted、unitConversion、reverse、multiplyDivide、plusMinusAverage、直接連線 | Driver（F-curve 關鍵格，不使用 Python 運算式，所以不需要開 Auto Run Scripts） |
| skinCluster | Vertex groups ＋ Armature modifier |
| blendShape | Shape keys（權重也可以被 driver 驅動） |
| 單位與座標軸 | Maya Y-up / 公分 → Blender Z-up / 公尺（比例可調） |

匯入完成後，所有無法完整轉換的地方都會寫進 Blender 文字區塊 `<rig名稱>_import_log`。

## 安裝

**Blender（4.2 以上）**

```bash
python tools/build_addon_zip.py        # 產生 dist/maya_rig_importer-0.1.0.zip
```

Blender → Edit → Preferences → Get Extensions → 右上角 ⌄ → **Install from Disk**，選這個 zip。
（舊版安裝方式：把 `blender_addon/maya_rig_importer` 資料夾複製到 Blender 的 `addons` 目錄。）

**Maya**

把 `maya_exporter/maya_rig_exporter.py` 放進 Maya 的 scripts 資料夾（例如 `文件/maya/scripts`）。

## 使用方式

### 1. 在 Maya 匯出

先把 rig 放回 bind pose / 預設姿勢（這個姿勢就是 Blender 的 rest pose），選取 rig 最上層的群組（骨架、控制器、模型都要包含），然後在 Script Editor（Python）：

```python
import maya_rig_exporter
maya_rig_exporter.show_ui()                                   # 開啟小視窗
# 或直接：
maya_rig_exporter.export_rig("D:/out/hero.m2b.json")          # 匯出目前選取
maya_rig_exporter.export_rig("D:/out/hero.m2b.json", roots=["rig_grp", "geo_grp"])
```

批次（mayapy，不開介面）：

```bash
mayapy maya_rig_exporter.py hero.ma hero.m2b.json --root rig_grp --root geo_grp
```

### 2. 在 Blender 匯入

File → Import → **Maya Rig (.m2b.json)**。選項：

- **Scale**：每 1 Maya 公分換成多少 Blender 單位（預設 0.01，也就是轉成公尺）。
- **Bone Axis**：Maya 關節哪一個軸當成 Blender 骨頭的方向。`Auto` 會讓每根骨頭指向它的子關節。
- **Meshes / Constraints & IK / Drivers / Control Shapes**：可以個別關掉。

匯入後的骨頭集合（Bone Collections）：

- `CTRL`：控制器（動畫師用的）
- `DEF`：有蒙皮的關節
- `JNT`：沒有蒙皮的關節（預設隱藏）
- `MCH`：群組、ikHandle 和轉換時產生的輔助骨（預設隱藏）

## 已知限制

- Spline IK、geometry / normal / tangent 約束、condition / remapValue 等節點還不支援，會列在 log 裡。
- parentConstraint 的部分軸向 skip、orientConstraint 的 skip 不支援（會整個套用）。
- 多目標 orient / point 約束以「疊加 influence」方式近似 Maya 的加權平均；權重總和為 1 時才精確。
- rotatePivot / scalePivot 不為 0 的控制器會以節點原點旋轉。
- BlendShape 的 in-between 目標尚未轉換。
- Rest pose 需要等於蒙皮的 bind pose；匯出器偵測到不一致時會警告（先執行 Skin → Go to Bind Pose）。
- Maya 匯出器沒辦法在沒有 Maya 的環境下自動測試，請回報遇到的錯誤訊息。

檔案格式見 [`docs/FORMAT.md`](docs/FORMAT.md)。

## 開發 / 測試

```bash
pip install pytest bpy      # bpy 版本要配合你的 Python（Python 3.13 → bpy 5.x）
python -m pytest tests
```

- `tests/test_core.py`：純 Python 的轉換核心（不需要 Blender）。
- `tests/test_blender_e2e.py`：用 `bpy` 實際建立一個測試手臂 rig，檢查 rest pose 不變、IK 跟隨控制器、SDK / BlendShape driver、space switch、point 約束 offset、aim、蒙皮變形。

架構：`core/plan.py` 把匯出的資料轉成與 Blender 無關的「建構計畫」（骨頭、約束、driver、mesh），
`builder.py` 只負責用 `bpy` 把計畫實際建出來，所以大部分轉換邏輯都能在沒有 Blender 的環境測試。
