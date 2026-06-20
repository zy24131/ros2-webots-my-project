---
name: webots-urdf-robot
description: >-
  修复和维护从 URDF/CAD 导入的 Webots 机器人项目（mesh 路径、boundingObject 规则、
  STL 预缩放、内联 wbt 无需 Convert、质量参数、船模/穿模、void 控制器、world 设置）。
  在编辑 protos/robot.proto、worlds/*.wbt、meshes/、robot.urdf 时使用。
---

# Webots URDF 机器人工作流

## 项目结构

```
my_project/
├── meshes/           # STL（已缩放到米）
├── protos/robot.proto
├── worlds/my_project.wbt
├── controllers/void/
└── robot.urdf
```

## 硬性约束（Webots R2025）

### boundingObject 规则

| 允许 | 禁止 |
|------|------|
| `Pose` + `Mesh` / `Box` / `Cylinder` / `Sphere` | `boundingObject` 内任何 `Transform` |
| `Group` 下直接放几何体 | `Pose` 里再嵌套 `Pose` |
| 单层 `Pose` + 几何体 | 在 `boundingObject` 里 `USE` 视觉 mesh 的 `DEF` |

常见报错与原因：

- `Invalid escaped character` → mesh URL 含 `\`；改为 `../meshes/foo.stl`
- `Cannot insert Transform node in boundingObject` → 去掉 `Transform` / `scale`
- `Cannot insert Pose node in children of Pose in bounding object` → 合并为单层 `Pose`
- Convert to node 后出现 `Duplicate field value: url` → 内存中 VRML 树损坏；从磁盘 Reload

### 机器人根位姿（本项目）

**除非用户明确要求，否则不要改。** 详见 `.cursor/rules/webots-robot-pose.mdc`：

```vrml
Robot {
  translation 0 -2.8549e-07 0.28
  rotation 1 0 0 -1.5707953071795862
}
```

## 工作流清单

```
- [ ] 1. Mesh 路径与单位
- [ ] 2. 视觉几何
- [ ] 3. 机身 boundingObject
- [ ] 4. 轮胎 boundingObject
- [ ] 5. 控制器 + 场景 + 内联 wbt
- [ ] 6. Webots 验证
```

---

## 步骤 1：Mesh 路径与单位

**路径**：STL 放在 `meshes/`。`protos/robot.proto` 和 `worlds/` 里都用 `../meshes/part_xxx.stl`。禁止 `meshes\`（Windows 反斜杠会触发转义错误）。

**单位**：CAD 导出的 STL 通常是 mm；Webots 用米。

推荐做法（本项目已采用）：

1. 一次性把所有 STL 顶点乘以 `0.001`（见 `scripts/scale_meshes.py`）
2. 从 `robot.urdf` 的 mesh 标签去掉 `scale="0.001 0.001 0.001"`
3. 从 proto/wbt 视觉 `Transform` 节点去掉 `scale 0.001 0.001 0.001`

**不要**在 `boundingObject` 里写 `scale`——那里禁止 `Transform`。

---

## 步骤 2：视觉几何

每个 link 零件的标准写法：

```vrml
Transform {
  translation T1x T1y T1z
  rotation 1 0 0 1.570796
  children [
    Shape {
      castShadows FALSE
      appearance PBRAppearance { ... }
      geometry DEF part_xxx Mesh {
        url "../meshes/part_xxx.stl"
      }
    }
  ]
}
```

- 视觉外层用 `Transform`（**无 scale**；STL 已预缩放）
- 轮胎 mesh（>21845 三角面）设 `castShadows FALSE`
- 外层旋转：绕 X 轴 90°（`1 0 0 1.570796`）

---

## 步骤 3：机身 link 的 boundingObject

低面数 link：**与视觉相同的外层 Pose**，直接放 `Mesh`：

```vrml
boundingObject Pose {
  translation T1x T1y T1z
  rotation 1 0 0 1.570796
  children [
    Mesh {
      url "../meshes/part_xxx.stl"
    }
  ]
}
```

mesh 原点和缩放正确时，虚线包围盒与视觉对齐。

---

## 步骤 4：轮胎 boundingObject

轮胎 STL（`part_004`、`007`、`012`、`013`）各约 23000 三角面。用 Mesh 做碰撞会产生 1000+ 接触点 → Webots 只保留 10 个 → 穿透、物理失效。

### 方向（重要）

**轮胎碰撞体与视觉使用相同的外层 Pose：绕 X 轴旋转 90°。**

```vrml
rotation 1 0 0 1.570796
```

- 轴向量 `(1, 0, 0)` = 绕 **X 轴**（用户所称的全局 X 方向）
- **不要**写成 `0 0 1 1.570796`（绕 Z 轴）——那是错误方案
- `translation` 与视觉的 `T1` **完全相同**，不要另算 bbox 偏移

### 当前方案（Mesh 碰撞，与 wbt 一致）

```vrml
boundingObject Pose {
  translation -0.900988 0.280422 -0.161446
  rotation 1 0 0 1.570796
  children [
    Mesh {
      url "../meshes/part_007_NAUO7.stl"
    }
  ]
}
```

四轮 link 的 `T1`：

| Link | Mesh 文件 | translation（T1） |
|------|-----------|---------------------|
| link_004 | part_007_NAUO7.stl | -0.900988 0.280422 -0.161446 |
| link_007 | part_004_NAUO4.stl | 0.36631 0.280422 -0.161446 |
| link_010 | part_012_NAUO12.stl | 0.36631 0.280422 1.326554 |
| link_013 | part_013_NAUO13.stl | -0.807213 0.280422 1.326554 |

### 可选方案（Cylinder 碰撞）

若需降低 contact 数量，**仍保持相同 Pose**（`T1` + 绕 X 轴 90°），只把 `Mesh` 换成 `Cylinder`：

```vrml
boundingObject Pose {
  translation -0.900988 0.280422 -0.161446
  rotation 1 0 0 1.570796
  children [
    Cylinder {
      radius 0.4475
      height 0.2150
    }
  ]
}
```

批量应用：`python scripts/apply_wheel_cylinders.py`（只改四个轮胎 link，Pose 与视觉对齐）。

**禁止**：

- `boundingObject Transform { scale ... }`
- 嵌套 `Pose` > `Pose` > `Cylinder`
- 把轮胎碰撞旋转改成绕 Z 轴（`0 0 1 1.570796`）

若用户要求轮胎继续用 Mesh 碰撞，尊重其选择，仅提示 contact 警告即可。

---

## 步骤 5：控制器与场景

**void 控制器**（`controller "void"` 需要）：

```
controllers/void/void.c
controllers/void/Makefile
→ 执行：make -C controllers/void
```

**地面**：默认 `RectangleArena` 为 1×1 m，机器人约 1.5×2.3 m，需放大：

```vrml
RectangleArena {
  floorSize 8 8
}
```

### 内联 wbt —— 不用 Convert to node

**当前 workflow**：`worlds/my_project.wbt` 里已是内联 `Robot { ... }`，**没有** `EXTERNPROTO robot.proto`。

- **直接打开 / Reload** `my_project.wbt` 即可仿真
- **不要**依赖 Convert to node（易出 `Duplicate url`、内存脏树）
- **改结构时以 wbt 为准**；`protos/robot.proto` 作备份/模板，需手动同步
- 若用户已在 Webots 里拖过机器人，wbt 根位姿可能与 rule 文件不同——**以 wbt 为准，勿擅自改回**

### 相对最初 URDF 导出 proto 的本质改动

| 最初（导出器输出） | 现在 |
|-------------------|------|
| `meshes\part_xxx.stl` | `../meshes/part_xxx.stl` |
| 视觉 `Transform { scale 0.001 }` | STL 预缩放 ×0.001，视觉**无 scale** |
| 碰撞 `Pose { USE part_xxx }`（无 scale，大 1000 倍） | 碰撞 `Pose { Mesh { url } }` |
| 机身 1000 kg / 轮胎 ~822 kg | 机身 **100 kg** / 轮胎 **10 kg**（惯性同比例缩放） |

碰撞体演变（均因 Webots 规则）：`USE` → `Transform+USE`（USE 禁止）→ `Transform+Mesh`（Transform 禁止）→ **`Pose+Mesh`（最终）**。

### 当前质量参数（`robot.urdf` / wbt / proto 同步）

| Link | 质量 |
|------|------|
| 机身 base / part_002 | 100 kg |
| 四轮 link_004/007/010/013 | 10 kg |
| 中间连杆 | 5~26 kg（URDF 原值，未改） |

改质量时同步更新 `inertiaMatrix` / URDF `<inertia>`。

---

## 步骤 6：Webots 验证

1. Reload `my_project.wbt`（**不要 Convert**）
2. 可选：Rendering → Show Bounding Objects
3. 轮胎 **Mesh** 碰撞：预期 contact 数量警告，可能出现**船模**（左右晃、弹、穿地）
4. 轮胎 **Cylinder**：接触稳定（用户可能拒绝，尊重选择）

### 船模现象（Mesh 轮胎接地）

- 每轮胎 ~23000 三角面 → 1000+ 接触点，ODE 只保留 ~10 个
- 接触点每帧乱跳 → 支撑不稳 → 像船一样晃
- 大质量会放大晃动（已通过减质量缓解）
- 根治：轮胎改 Cylinder 碰撞；缓解：WorldInfo 阻尼/contactProperties

用户已手动调整、勿擅自还原的内容：

- 机器人根位姿（用户在 wbt 里拖过后以 wbt 为准）
- Viewpoint 相机
- Webots 保存格式（`url [ ... ]`、`Transform` vs `Pose`）

---

## 辅助脚本

| 脚本 | 作用 |
|------|------|
| `scripts/scale_meshes.py` | 把 `meshes/*.stl` 全部 ×0.001（mm→m） |
| `scripts/apply_wheel_cylinders.py` | 四个轮胎改为 Cylinder，Pose 与视觉相同（绕 X 轴） |
| `scripts/compute_wheel_pose.py` | 打印四轮 Pose（T1 + 绕 X 轴 90°） |

在项目根目录执行：

```bash
python3 .cursor/skills/webots-urdf-robot/scripts/scale_meshes.py
python3 .cursor/skills/webots-urdf-robot/scripts/compute_wheel_pose.py
python3 .cursor/skills/webots-urdf-robot/scripts/apply_wheel_cylinders.py
```

---

## 决策指南

```
mesh URL 解析报错？
  → 正斜杠，../meshes/

包围盒比视觉大 1000 倍？
  → STL 未缩放，或 scale 只在视觉上、碰撞体没有

boundingObject 被跳过 / INFO "child expected"？
  → 去掉 Transform、嵌套 Pose 或 USE

轮胎穿地 / 船模 / 1000+ contact 警告？
  → 高面数 Mesh 碰撞；可选 Cylinder（步骤 4）；或调质量/阻尼

不用 Convert？
  → 直接 Reload my_project.wbt；wbt 内联 Robot，proto 仅备份

改 proto 还是 wbt？
  → 仿真以 wbt 为准；改完同步 proto

用户说「别改了」「可以了」？
  → 停止；只改用户明确要求的项

用户要求保持机器人位姿？
  → 绝不改 Robot 根节点 translation/rotation
```
