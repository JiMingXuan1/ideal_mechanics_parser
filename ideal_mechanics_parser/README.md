# Ideal Mechanics Parser

**物理竞赛级二维理想力学模拟器**——拖拽搭建、实时仿真，零代码做物理。

在后端，它是 SymPy 符号推导 + SciPy Radau 隐式积分 + 最大坐标法的多体动力学引擎；在前端，它是纯粹的 HTML5 Canvas + Vanilla JS，零框架依赖。

---

## 快速开始

```bash
cd ideal_mechanics_parser
pip install -r requirements.txt
python server.py
```

浏览器打开 `http://localhost:8000`。

---

## UI 速览

```
┌────────────────────────────────────────────────┐
│  [↖] [⊕] [◫] [⊡] [╳] [✕]          [▶ Run]   │  工具栏（左上）
│  ────────────────────────                       │
│                                                 │
│                   画布                          │  拖拽、点击构建
│           (Canvas)                              │
│                                                 │
│                                    ┌──────────┐│
│                                    │ ID: n1   ││  属性面板（右下）
│                                    │ Type:    ││
│                                    │ m: [1.0] ││
│                                    │ ...      ││
│                                    └──────────┘│
│                                    ┌──────────┐│
│  ══════════════════════════════════│ > gamemode││  控制台（底部）
│                                    └──────────┘│
└────────────────────────────────────────────────┘
```

### 工具栏

| 按钮 | 快捷键 | 功能 |
|------|--------|------|
| `↖` Select | `V` | 选择/拖拽实体 |
| `⊕` Point | `P` | 放置质点 |
| `◫` RigidBody | `B` | 放置刚体（矩形，带朝向） |
| `⊡` Anchor | `A` | 放置固定锚点 |
| `╳` Edge | `E` | 连接两个节点（选类型） |
| `✕` Delete | `Del` | 删除选中 |
| `▶` / `⏸` Run | — | 开始/暂停仿真 |
| `⏹` Stop | — | 停止仿真（运行时出现） |

### 创建仿真：3 步

**1. 放节点**：点 `⊕` 放质点，点 `⊡` 放锚点
**2. 连边**：点 `╳`，依次点两个节点，弹出类型选择（杆/弹簧/轨道/铰接…）
**3. 运行**：点 `▶`，实时看到物理运动

### 属性面板

选中任意实体或边后，右下角出现属性面板。可修改：

- **质点**：`m`（质量）、`x/y`（初始位置）、`vx/vy`（初始速度）
- **刚体**：`m`（质量）、`shape`（rect/rod）、`length`、`width`、`x/y/theta`、`vx/vy/omega`
- **锚点**：`x/y`
- **边**：根据类型显示不同参数（杆长、弹簧系数、铰接偏移……）。连接刚体时出现 `from_pivot[u/v]` / `to_pivot[u/v]` 附着点偏移输入。

### 控制台

按下 **`` `**（反引号）打开/关闭控制台。支持命令：

| 命令 | 说明 |
|------|------|
| `gamemode XZ` | 切换到竖直平面（重力开） |
| `gamemode XY` | 切换到水平平面（重力关） |
| `gamemode g` 或 `gravity universal on/off` | 切换万有引力（G=1 demo 模式） |
| `speed <factor>` | 设置播放速度（1=正常，10=10倍） |
| `trails on/off` | 显示/隐藏运动轨迹 |
| `duration <sec>` | 设置仿真时长（秒） |
| `run` | 开始仿真 |
| `stop` | 停止仿真 |
| `set n1 x 5` | 设置实体 `n1` 的属性 `x` 为 5 |
| `undo` | 撤销上一步 |
| `redo` | 重做 |
| `help` | 查看所有命令 |

---

## 实体库

### 节点

| 类型 | 图标 | 自由度 | 说明 |
|------|------|--------|------|
| **Anchor** | 菱形 + 十字 | — | 固定锚点，不可移动 |
| **MassPoint** | 蓝圆 | `(x, y)` | 质点，受重力/约束 |
| **RigidBody** | 矩形/线段 + 方向线 | `(x, y, θ)` | 刚体，支持转动和铰接。`I` 自动计算（匀质杆 `¹⁄₁₂mL²`、矩形 `¹⁄₁₂m(L²+W²)`） |

### 边

| 类型 | 连线 | 说明 | 参数 |
|------|------|------|------|
| **IdealRod** (轻杆) | 实线 + label | 两节点距离恒定：`‖p₁-p₂‖² = L²` | `length` |
| **IdealSpring** (弹簧) | 锯齿线 | 弹性势能：`½k(d-l₀)²` | `k`（刚度）, `l0`（原长） |
| **SmoothRail** (轨道) | 虚线 | 约束在曲线 `f(x,y,t)=0` 上 | `expr`（SymPy 表达式） |
| **FixedCoordinate** (定坐标) | 圆 + 十字 | 锁定单坐标 `x=c` 或 `y=c` | `coord`（x/y）, `value` |
| **LinearRelation** (线性关系) | 虚线 | 多坐标线性组合=常数 | `coeffs`, `constant` |
| **DistanceSum** (滑轮绳) | 虚线 + 过滑轮 | 总绳长恒定 | `via_id`（滑轮节点）, `length` |
| **AngleConstraint** (定角) | 实线 + 角度 | 连线与水平夹角固定 | `angle`（弧度） |
| **HingeJoint** (铰接) | ⭕ | 刚体局部点→世界点/另一节点 | `pivot`, `world`/`to` |
| **SoftRope** (软绳) | — | 松弛→绷紧自动切换（实验性） | `length` |

### 附着点

边连接刚体时，可以在属性面板中指定 `from_pivot[u/v]` / `to_pivot[u/v]`（局部坐标偏移）。附着点以紫色圆点显示在刚体上，支持拖拽。

碰撞检测支持 **质点到杆** 和 **杆到杆** 模式，碰撞冲量包括角动量效应（r × J → Δω）。

---

## 表达式注入（逃生舱）

核心特性：在数字字段填入 **数学函数表达式**（不是数字），后端用 SymPy 安全解析。

### 移动锚点

锚点属性面板中的 `ƒ x_expr` 和 `ƒ y_expr` 接受时间函数：

```
x_expr: "t**2"          → 锚点按 x = t² 运动
y_expr: "5.0"           → y 固定为 5.0
x_expr: "2*sin(pi*t)"   → 锚点简谐运动
```

锚点位置由表达式决定，与之相连的杆/弹簧/轨道自动跟随。

### 外部驱动力

质点属性面板中的 `ƒ Fx(t)` 和 `ƒ Fy(t)` 给质点施加与时间相关的外力：

```
Fx(t): "10*sin(2*pi*t)"   → 周期驱动力
Fy(t): "m*g"              → 抵消重力（m=质量，g=9.81）
Fx(t): "5.0"              → 恒力 5N
```

力是**广义力**，直接作用在对应坐标方向。（`XZ` 平面下重力默认向下—Y 方向，所以 `Fy(t) = m*g` 可抵消重力）

### 光滑轨道

连接节点的 SmoothRail 边，`expr` 参数接受隐式曲线方程：

```
y - x**2         → 抛物线轨道
x**2 + y**2 - 4 → 半径 2 的圆轨道
y - sin(x)      → 正弦波轨道
```

轨道方程可以使用 `x`、`y`（质点的坐标）和 `t`（时间）。

### 可用函数

```
sin  cos  tan  pi  exp  sqrt  Abs  log
asin  acos  atan  sinh  cosh  tanh
t  (时间变量)
```

---

## JSON 拓扑格式（高级）

启动服务器后，也可通过 `POST /solve` 直接发 JSON 批量求解：

```json
{
  "system_env": {
    "view_plane": "XY",
    "gravity": 9.81,
    "time_step": 0.01,
    "duration": 10.0
  },
  "nodes": [
    {"id": "n1", "type": "Anchor", "init_pos": [0, 0]},
    {"id": "n2", "type": "MassPoint",
     "params": {"m": 1.0, "external_force_x_expr": "5*sin(t)"},
     "init_state": {"x": 2, "y": 0, "vx": 0, "vy": 0}}
  ],
  "edges": [
    {"id": "e1", "type": "IdealRod", "from": "n1", "to": "n2",
     "params": {"length": 2.0}}
  ]
}
```

返回格式：

```json
{
  "t": [0.0, 0.01, 0.02, ...],
  "q": [[x0, y0, x1, y1, ...], ...],
  "qd": [[vx0, vy0, vx1, vy1, ...], ...],
  "node_order": ["n1", "n2"],
  "body_dofs": [2, 2]
}
```

`body_dofs` 指示每个节点在 `q` 中占几个自由度（质点=2，刚体=3）。

命令行求解：

```bash
python main.py examples/single_pendulum.json
python main.py examples/hinged_rod.json
```

---

## 示例

| 文件 | 说明 |
|------|------|
| `examples/single_pendulum.json` | 单摆 |
| `examples/double_pendulum.json` | 双摆（混沌） |
| `examples/spring_oscillator.json` | 弹簧振子 |
| `examples/atwood.json` | 阿特伍德机 |
| `examples/hinged_rod.json` | 复合摆（刚体+铰接） |

---

## 架构概览

```
JSON 拓扑 → Step 1: 符号实例化 → Step 1.5: N-R 投影
         → Step 2: 能量聚合 (T + V) → Step 3: 约束收割
         → Step 4: Baumgarte + Radau 积分 → 时间序列 JSON
```

- **最大坐标法**：每个质点 `(x, y)` 是独立自由度，约束通过拉格朗日乘子法处理
- **拉格朗日方程**：`L = T - V` → `LagrangesMethod` 自动推导运动方程
- **Baumgarte 稳定化**：`α=β=1` 消除约束漂移
- **Radau IIA**：5 阶隐式 Runge-Kutta，适合 DAE 系统

---

## 运行测试

```bash
pytest tests/ -v
```

当前 **63 个测试**全部通过，覆盖：单摆周期、弹簧频率、双摆守恒、约束漂移、投影收敛、奇异 Jacobian、表达式注入、碰撞检测、软绳绷紧、角动量守恒、刚体铰接、API 端到端。

---

## License

MIT
