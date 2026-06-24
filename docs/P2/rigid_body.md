# 二维刚体动力学 (2D Rigid Body Dynamics)

## 问题

Phase 1 仅支持质点（2 个平动自由度）。物理竞赛中大量问题涉及刚体（杆、圆盘、方块）的旋转行为，需要引入角自由度 $\theta$、转动惯量 $I$ 以及铰接约束。

## 刚体模型

### 自由度

每个刚体贡献 3 个广义坐标：

```
q_i = [x_i, y_i, θ_i]    位置 + 绕质心转角
qd_i = [vx_i, vy_i, ω_i]  平动速度 + 角速度
```

### 动能

总动能为平动 + 转动：

```
T_i = ½ m_i (vx_i² + vy_i²) + ½ I_i ω_i²
```

$I_i$ 为**绕质心**的转动惯量。对于通过质心的轴，$I_{cm}$ 由前端传入（或通过几何公式自动计算）。

### 势能

重力势能与质点一致（仅 XZ 平面生效）：

```
V_i = m_i g · y_i
```

### 惯量约定

| 形状 | 公式 | 参数 |
|------|------|------|
| 矩形（绕质心） | $I = \frac{1}{12}m(w^2 + h^2)$ | `width`, `height` |
| 细杆（绕一端） | $I = \frac{1}{3}mL^2$ | `length`, `hinge_at_end: true` |
| 细杆（绕质心） | $I = \frac{1}{12}mL^2$ | `length` |
| 圆盘 | $I = \frac{1}{2}mR^2$ | `radius` |
| 用户指定 | $I =$ 直接值 | `I` |

前端在新建 RigidBody 时弹出形状选择对话框，根据形状自动填入惯量。

## 铰接约束 (HingeJoint)

### 问题描述

铰接约束将刚体上的**某个点**固定到另一个物体（或世界坐标）上。该点可以是质心、端点或任意偏移位置。

### 约束方程

刚体 $i$ 上的局部点 $(u_i, v_i)$（相对于质心）的世界坐标：

```
P_i_world = (x_i + u_i·cosθ_i - v_i·sinθ_i,  y_i + u_i·sinθ_i + v_i·cosθ_i)
```

铰接到世界点 $(x_0, y_0)$ 的两个约束方程：

```
f₁ = x_i + u_i·cosθ_i - v_i·sinθ_i - x_0 = 0
f₂ = y_i + u_i·sinθ_i + v_i·cosθ_i - y_0 = 0
```

### 两种铰接模式

**模式 A：刚体 → 世界固定点**

```
{ "type": "HingeJoint", "from": "b1", "params": { "pivot": [0, 0], "world": [5.0, 3.0] } }
```

`pivot` 为刚体局部坐标中的铰接点偏移量（相对于质心），`world` 为世界坐标中的固定位置。

**模式 B：刚体 ↔ 刚体**

```
{ "type": "HingeJoint", "from": "b1", "to": "b2", "params": { "pivot_a": [0, -0.5], "pivot_b": [0, 0.5] } }
```

将 `b1` 上的局部点 `pivot_a` 与 `b2` 上的局部点 `pivot_b` 重合。生成两个约束方程：

```
f₁ = (b1_x + u₁·cosθ₁ - v₁·sinθ₁) - (b2_x + u₂·cosθ₂ - v₂·sinθ₂) = 0
f₂ = (b1_y + u₁·sinθ₁ + v₁·cosθ₁) - (b2_y + u₂·sinθ₂ + v₂·cosθ₂) = 0
```

### 刚体 ↔ 质点铰接

简化版：刚体上的局部点铰接到 MassPoint 的 (x, y)：

```
{ "type": "HingeJoint", "from": "b1", "to": "m1", "params": { "pivot_a": [0, -0.5] } }
```

一个约束方程（因为质点只有平动自由度直接匹配）：

```
f₁ = (b1_x + u₁·cosθ₁ - v₁·sinθ₁) - m1_x = 0
f₂ = (b1_y + u₁·sinθ₁ + v₁·cosθ₁) - m1_y = 0
```

## JSON 拓扑格式

```json
{
  "nodes": [
    {
      "id": "b1",
      "type": "RigidBody",
      "params": {
        "m": 2.0,
        "I": 0.1667,
        "shape": "rod",
        "length": 2.0
      },
      "init_state": {
        "x": 0.0, "y": 0.0, "theta": 0.0,
        "vx": 0.0, "vy": 0.0, "omega": 0.0
      }
    }
  ],
  "edges": [
    {
      "id": "h1",
      "type": "HingeJoint",
      "from": "b1",
      "params": {
        "pivot": [0, -1.0],
        "world": [0.0, 0.0]
      }
    }
  ]
}
```

## 实现变更

### 后端

| 文件 | 变更 |
|------|------|
| `entities/rigid_body.py` | 新建 RigidBody 类，存储 m, I, shape params |
| `entities/hinge_joint.py` | 新建 HingeJoint 边类型 |
| `core/symbols.py` | 新增 `add_rigid_body()`，分配 (x, y, θ) 三个符号，`nq += 3`；`get_q0`/`get_qd0` 支持 theta/omega |
| `core/energy.py` | `assemble_energy` 遍历刚体，累加 ½Iω² |
| `core/constraints.py` | `harvest_constraints` 处理 `HingeJoint`：旋转矩阵映射局部→世界坐标 |
| `core/engine.py` | `_step1_instantiate` 实例化 RigidBody |
| `io_handler/parser.py` | `VALID_NODE_TYPES` 增加 `"RigidBody"`，`VALID_EDGE_TYPES` 增加 `"HingeJoint"` |

### 前端

| 文件 | 变更 |
|------|------|
| `physics/GraphBuilder.js` | 增加 `RigidBody` 分支，输出 type + params + init_state |
| `canvas/Entities.js` | 新增 `drawRigidBody(ctx, x, y, theta, ...)` — 绘制带朝向标记的矩形 |
| `canvas/Entities.js` | 新增 `drawHingeJoint` — 绘制铰接点圆圈 |
| `ui/PropertiesPanel.js` | 新增 RigidBody 属性编辑：m, I, shape, theta, omega |
| `core/StateMachine.js` | 确保 `_addEntity` 支持 `RigidBody` |

## 限制

- 刚体始终是二维的，不支持离面旋转（无 3D 陀螺效应）
- 铰接点偏移量 `pivot` 在刚体局部坐标系中固定，不可随时间变化
- 不支持刚体之间 flexible 连接（P3 弹性铰链）
