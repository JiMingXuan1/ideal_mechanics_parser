初始状态规范与表达式注入 (P1 Features Specification)

本说明文档规定了引擎在解析用户自定义状态和底层表达式时的绝对边界。任何负责编写 `Topology Parser` 和 `Symbolic Engine` 的代码都必须严格遵循此规范。

## 1. 严格的初始状态定义 (Strict Initial States)

**物理法则红线**：系统的初始状态只能由**位置 (Position)** 和**速度 (Velocity)** 唯一确定。**绝对禁止在前端输入或后端解析中暴露任何形式的“初始加速度”参数！** 加速度是由动力学方程在任意时间步严格计算得出的结果，将其作为初始自由变量会导致系统动力学逻辑彻底崩溃。

# 规范数据结构
在节点 (Nodes) 的 `init_state` 字段中，仅允许出现以下键值：
* 质点 (MassPoint): `x`, `y`, `vx`, `vy`
* 刚体 (RigidBody - Phase 2 预留): 增加 `theta`, `omega` (角速度)

**示例：**
json
"init_state": {
  "x": 0.0,
  "y": 5.0,
  "vx": 2.0,  // 正确：提供初始水平速度
  "vy": 0.0
  // 严禁添加 "ax" 或 "ay"
} 

## 2. 逃生舱：自定义表达式注入 (Expression Injection)

为应对物理竞赛中普遍存在的变质量系统、受迫振动和非定常约束（含时约束），系统必须提供一个允许用户绕过预设图元、直接注入数学公式的“逃生舱 (Escape Hatch)”机制。
解析规范前端允许在某些特定的 params 字段中传入字符串格式的数学表达式，后端必须安全、准确地将其转化为 SymPy 的符号对象。
全局时间符号：后端在初始化 SymPy 环境时，必须全局定义唯一的独立时间变量 $t$ (t = sympy.Symbol('t'))。
安全解析（沙盒强制协议）：当解析带有 `_expr` 或 `expr` 后缀的参数时，必须调用 `sympy.sympify()` 并遵循以下铁律：
1. **禁用 eval**：设置 `sympify(..., evaluate=False)`。
2. **受限 locals_dict**：仅传入 `{'t': t, 'sin': sympy.sin, 'cos': sympy.cos, 'tan': sympy.tan, 'pi': sympy.pi, 'exp': sympy.exp, 'sqrt': sympy.sqrt, 'Abs': sympy.Abs}`，**绝对禁止传入 `__builtins__`**。
3. **白名单校验**：解析后调用 `.free_symbols` 检查是否引入了未注册的危险符号变量，一旦发现立即抛出 `SecurityError`。

* 应用场景示例场景 A：非定常约束 (移动的锚点)例如一个水平向右以恒定加速度 $a=2$ 运动的墙壁/悬挂点：JSON{
  "id": "n1", 
  "type": "Anchor", 
  "params": {
    "x_expr": "0.5 * 2 * t**2",  // 解析为 t 的函数
    "y_expr": "10.0"             // 静态解析为浮点数
  }
}
场景 B：随时间变化的系统参数 (受迫驱动力 / 变质量)例如一个在系统中受到特定周期性驱动力的质点，直接将表达式注入其广义力或势能场：JSON{
  "id": "n2",
  "type": "MassPoint",
  "params": {
    "m": "1.0",
    "external_force_x_expr": "5.0 * sin(2 * pi * t)" // 强制驱动力
  },
  "init_state": {"x": 0, "y": 0, "vx": 0, "vy": 0}
}

## 3. 高级约束类型规范 (Advanced Constraints Specification)

除光滑轨道外，引擎还支持以下等式约束图元。

### 3.1 固定坐标 (FixedCoordinate)

锁定单个自由度，将质点约束在某坐标值上。适用于滑块、导杆等场景。

```json
{"id": "c1", "type": "FixedCoordinate", "from": "n2", "params": {"coord": "x", "value": 3.0}}
```
约束方程：$x_2 - 3.0 = 0$

`coord` 可取 `"x"` 或 `"y"`，`value` 为浮点数值。

### 3.2 线性关系 (LinearRelation)

广义一次齐次线性约束，连接两质点的坐标。

```json
{"id": "c2", "type": "LinearRelation", "from": "n2", "to": "n3", "params": {"coeffs": [1, -1, -1, 1], "constant": 0}}
```
约束方程：$x_2 - y_2 - x_3 + y_3 = 0$

`coeffs` 为 $[a_1, b_1, a_2, b_2]$ 对应 $a_1 x_1 + b_1 y_1 + a_2 x_2 + b_2 y_2 + c = 0$。

`to` 可省略（单点约束），省略时方程退化为 $a_1 x_1 + b_1 y_1 + c = 0$。

### 3.3 距离和约束 / 滑轮约束 (DistanceSum)

阿特伍德机核心约束。两质点通过一个公共点（滑轮）相连，总绳长恒定。

```json
{"id": "c3", "type": "DistanceSum", "from": "m1", "to": "m2", "params": {"via_id": "pulley", "length": 9.0}}
```
约束方程：$\|p_{m1} - p_{pulley}\| + \|p_{m2} - p_{pulley}\| - 9.0 = 0$

`via_id` 引用一个 Anchor 或 MassPoint 作为滑轮的悬挂点。数值验证表明该约束在 8 秒积分内能量漂移 < 1e-12、约束误差 < 1e-10。

### 3.4 角度约束 (AngleConstraint)

固定两质点连线与水平方向的夹角。

```json
{"id": "c4", "type": "AngleConstraint", "from": "n2", "to": "n3", "params": {"angle": 1.5708}}
```
约束方程：$(x_3-x_2)\sin\theta - (y_3-y_2)\cos\theta = 0$

`angle` 为弧度值。1.5708 rad ≈ 90°（竖直）。该约束将连线方向锁定，不约束距离。

## 4. 光滑轨道规范 (Smooth Rails Specification)

光滑轨道复用逃生舱的表达式注入机制，将轨道定义为一条隐式解析曲线。

### 数据格式
在 `MassPoint` 的 `params` 中传入轨道约束表达式：
```json
{
  "id": "n3",
  "type": "MassPoint",
  "params": {
    "m": 1.0,
    "rail_expr": "y - x**2"
  },
  "init_state": {"x": 0, "y": 0, "vx": 0, "vy": 0}
}
```

### 解析规则
* 后端将 `"rail_expr"` 中的字符串传入 `sympify(locals_dict=SAFE_LOCALS, evaluate=False)`，产生符号约束方程 $f(x, y) = 0$。
* 该约束自动并入 Step 3 的约束列表 $f_c = 0$，与杆件约束同等处理（含 Baumgarte 稳定化）。
* 轨道方程可以是任何解析可微的隐式函数 $f(x, y) = 0$，不限于显式 $y = f(x)$。

## 5. 时间步长真相 (The Truth About time_step)

前端 JSON 中的 `time_step` 参数**仅决定返回给渲染层的坐标点密度**，绝不影响积分精度。

* `solve_ivp` 使用自适应步长控制（通过 `atol`、`rtol`），内部步长由误差阈值动态决定。
* 前端传入的 `time_step` 被映射为 `t_eval = np.arange(0, duration, time_step)`，告知求解器需要在这些时间点输出结果。
* 若 `time_step` 小于求解器自适应步长，求解器仍会按误差需求积分，仅通过插值输出对应时刻的值。
* 建议 `time_step` 值：动画渲染取 `0.016`（≈60fps），数据取证取 `0.001` ~ `0.01`。