# 理想力学解析器 (Ideal Mechanics Parser) - Core Design Memo

## 1. 项目定位 (Project Overview)
**这不是一个普通的实时游戏物理引擎！** 本项目是一个专为“物理竞赛级”理想力学问题设计的**拓扑建模与数值解算系统**。
系统彻底抛弃了传统游戏引擎中基于冲量和显式欧拉法的粗糙迭代，采用**基于分析力学的符号推导**与**高阶隐式数值积分**，旨在解决含有复杂理想约束（如无质量轻杆、多自由度耦合）的刚性动力学方程。

## 2. 核心架构 (Core Architecture)
系统采用严格的前后端分离与数据驱动架构 (Data-Driven Architecture)：

* **前端交互与渲染层 (HTML5 Canvas + JS)**
    * **职责**：提供可视化的图元沙盒。用户通过拖拽节点（质点、锚点）和连线（杆、弹簧）构建物理系统的拓扑图。
    * **限制**：前端**绝对不**参与任何微积分或受力运算。仅在运行前执行基础的图论算法（检测孤岛节点、非法拓扑闭环），随后将系统状态序列化为 JSON 抛给后端。
    * **渲染**：接收后端返回的时间序列矩阵，按帧插值重绘坐标点和线段。
* **后端解算引擎 (Python: SymPy + SciPy)**
    * **职责**：纯粹的“无头引擎 (Headless Engine)”。接收拓扑 JSON，基于**最大坐标法 (Maximal Coordinates)** 自动组装拉格朗日量和约束方程，并执行高精度数值求积。

## 3. 物理与数学核心 (Mathematical Core)
* **坐标系选择**：采用笛卡尔最大坐标法。每个质点保留绝对坐标 $(x, y)$，不使用内禀/极坐标降维，以应对前端不可预知的任意拓扑连结。
* **符号推导 (Symbolic Engine)**：使用 `sympy.physics.mechanics`。
    * 系统拉格朗日量：$L = T - V$
    * 所有“边”转化为代数约束方程 $f(x_i, y_i) = 0$。
    * 通过拉格朗日乘子法 $\lambda$ 自动推导欧拉-拉格朗日方程。
* **数值积分 (Numerical Solver)**：
    * 将符号推导出的常微分方程组 (ODE) 或微分代数方程组 (DAE) 转化为 NumPy 矩阵运算。
    * 使用 `scipy.integrate.solve_ivp`，严格指定 `method='Radau'` 或 `method='BDF'` 等隐式算法，以应对理想约束带来的**刚性方程 (Stiff Equations)** 导致的数值爆炸。
* **数值装甲三件套 (Numerical Armor Triad)**：为确保长期积分的数值稳定性，P1 就必须植入三道防线：
    1. **流形投影 (Manifold Projection)**：在积分启动前用 Newton-Raphson 迭代将用户输入的初始坐标 $q_0$ 强制投影到约束流形上，防止初始违反约束导致崩溃。
    2. **Baumgarte 稳定化 (Baumgarte Stabilization)**：在加速度层用弹簧-阻尼方程 $\ddot{f}_c + 2\alpha\dot{f}_c + \beta^2 f_c = 0$（默认 $\alpha=\beta=1$，可调超参数）替代纯刚性约束 $\ddot{f}_c=0$，消除约束漂移。
    3. **SVD 伪逆降维 (SVD Pseudo-Inverse)**：当冗余约束导致 Jacobian 矩阵 $J$ 秩亏时，使用 `np.linalg.pinv` 求解拉格朗日乘子，避免奇异矩阵崩溃。

## 4. MVP 第一期演进路线 (Phase 1 Roadmap)
第一期（最低可行性产品）仅支持**等式约束 (Bilateral Constraints)**，暂时禁止引入绳子等不等式约束。

**可用图元库 (Entity Library)：**
1. **固定锚点 (Anchor)**：提供绝对的基准坐标 $(x_0, y_0)$，无自由度。
2. **质点 (Mass Point)**：提供平动自由度 $(x, y)$ 与质量 $m$。
3. **理想弹簧 (Ideal Spring)**：势能连接件。向系统引入弹性势能项 $V_k = \frac{1}{2}k(l - l_0)^2$。
4. **理想轻杆 (Ideal Rod)**：刚性连接件。向系统引入几何等式约束 $(x_1 - x_2)^2 + (y_1 - y_2)^2 - L^2 = 0$。
5. **光滑轨道 (Smooth Rail)**：路径限制件。将质点约束在特定的解析几何曲线上。轨道方程由 `params` 中的 `"expr": "y - x**2"` 隐式表达式字符串定义，经沙盒化 `sympify` 解析（同 p1_features.md 逃生舱规范）。
6. **固定坐标 (FixedCoordinate)**：锁定单个自由度 $x_i = c$ 或 $y_i = c$。适用于滑轨、导向等场景。
7. **线性关系 (LinearRelation)**：广义线性约束 $a_1 x_1 + b_1 y_1 + a_2 x_2 + b_2 y_2 + c = 0$。
8. **距离和约束 (DistanceSum)**：滑轮/绳式约束 $\|p_1 - p_{via}\| + \|p_2 - p_{via}\| = L$。阿特伍德机。
9. **角度约束 (AngleConstraint)**：固定两点连线方向角 $(x_2-x_1)\sin\theta - (y_2-y_1)\cos\theta = 0$。

## 5. 投影平面切换 (Dimensionality Equivalent Switch)
为最大化引擎适用范围，系统支持伪 3D 的平面切换功能（在 JSON 头部定义 `view_plane`）：
* **`XY` 水平面模式**：俯视桌面模式。自动屏蔽重力项（设 $g=0$），Y 轴代表水平方向。为后期引入恒定大小的滑动摩擦力耗散函数（基于 $N=mg$）提供底层数学接口。
* **`XZ` 竖直面模式**：标准的重力场模式。X 为水平方向，Y 代表竖直方向（Z 轴降维）。所有的质量节点自动引入重力势能 $V = mgy$（此处 y 视为竖直坐标）。

## 6. 通信协议标准 (JSON Protocol Spec)
前后端通过标准 JSON 进行数据握手。以下为标准输入拓扑结构示例：

```json
{
  "system_env": {
    "view_plane": "XY",
    "gravity": 9.81,
    "time_step": 0.01,
    "duration": 10.0
  },
  "nodes": [
    {"id": "n1", "type": "Anchor", "init_pos": [0, 5]},
    {"id": "n2", "type": "MassPoint", "params": {"m": 1.0}, "init_pos": [3, 1]}
  ],
  "edges": [
    {"id": "e1", "type": "IdealRod", "from": "n1", "to": "n2", "params": {"length": 5.0}}
  ]
}