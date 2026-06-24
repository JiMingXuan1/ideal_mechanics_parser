# Memo 2: Python Backend Pipeline (符号推导与解算流水线)

这份文档规定了 Python 后端接收到 JSON 后，必须经历的 5 个严格流水线加工步骤。**严禁跳过任何一步！**

## Step 1: 拓扑实例化与符号注册 (Instantiation & Symbol Registration)
接收到前端传来的 JSON 后，必须立即构建对应的对象。
* 遍历 `nodes` 列表，每遇到一个 `MassPoint`，必须通过 `sympy.physics.mechanics.dynamicsymbols` 生成对应的时间函数 $x_i(t)$ 和 $y_i(t)$，以及它们的一阶导数（速度）。
* 将这些符号变量绑定在对象自身的属性上，绝对不可弄乱变量的下标。

## Step 1.5: 流形投影校验 (Manifold Projection)
绝对不要信任用户输入的初始坐标！在进行能量聚合之前，必须先通过 Newton-Raphson 迭代将 $q_0$ 投影到约束流形 $f(q)=0$ 上。
* 构建约束向量 $f(q)$，计算 Jacobian $J_f = \partial f / \partial q$。
* 迭代公式 $q^{(k+1)} = q^{(k)} - J_f^+(q^{(k)}) f(q^{(k)})$，其中 $J_f^+$ 是 Moore-Penrose 伪逆（`np.linalg.pinv`）。
* 收敛判据：$||f(q)|| < 10^{-12}$，最大迭代次数 $N_{max}=50$。
* 若迭代不收敛，抛出 `TopologyError` 异常，前端应显示"初始拓扑不合法！"错误。
* 投影结束后，用校正后的 $q_0$ 替换用户输入的原始坐标。

## Step 2: 能量聚合 (Energy Assembly)
此阶段仅负责系统能量池的标量累加，不进行受力分析。
* **动能池 ($T$)**：遍历所有 `MassPoint` 节点，累加系统的总动能：
  $$T = \sum \frac{1}{2} m_i (\dot{x}_i^2 + \dot{y}_i^2)$$
* **势能池 ($V$)**：
  * 检查 JSON 头部的 `view_plane`。**注意：**XY 为水平面无重力，XZ 为竖直面有重力。若为 `XZ` 模式，给每个质点累加 $V_g = m_i g y_i$（y_i 代表竖直坐标）；若为 `XY` 模式，直接忽略重力势能。
  * 遍历所有的弹性边（如 `IdealSpring`），根据其连接的两个节点的坐标，累加弹性势能 $V_e = \frac{1}{2} k (d - l_0)^2$，其中 $d$ 是两点欧氏距离。

## Step 3: 约束收割 (Constraint Harvesting)
最大坐标法的核心步骤，将几何拓扑转换为代数约束。
* 创建空约束列表 `holonomic_constraints = []`。
* 遍历所有刚性边（如 `IdealRod` 或 `SmoothRails`）。
* 提取其连接的两个节点坐标 $(x_a, y_a)$ 和 $(x_b, y_b)$，将其转化为等式为 $0$ 的表达式并压入列表：
  $$f_c = (x_a - x_b)^2 + (y_a - y_b)^2 - L^2$$

## Step 4: 数值组装与稳定化积分 (Numerical Assembly & Stabilized Integration)
这是整个引擎最核心的步骤。不能直接使用 `form_lagranges_equations()` 输出的 ODE，必须按以下子步骤进行数值处理：

### 子步骤 4.1: 符号矩阵提取 (Symbolic Matrix Extraction)
使用 `LagrangesMethod` 建立拉格朗日量 $L = T - V$ 和约束方程组。**但不要调用 `form_lagranges_equations()`**，而是提取以下符号组件：
* **质量矩阵 $M$**：从动能 $T$ 推导的 $n \times n$ 对称矩阵
* **力向量 $F$**：包含 Coriolis、重力、弹性力等的广义力向量
* **约束 Jacobian $J$**：约束方程 $f(q)$ 对广义坐标 $q$ 的 Jacobian 矩阵 $J = \partial f / \partial q$
* **约束 Hessian 项 $\dot{J}\dot{q}$**：用于加速度层约束的补偿项

### 子步骤 4.2: 数值编译 (Lambdify Compilation)
**必须**使用 `sympy.lambdify` 将 $M$、$F$、$J$、$f$ 和 $\dot{J}\dot{q}$ 全部编译为 NumPy 函数。严禁在积分循环中解析符号表达式！输出：
* `M_func(q)` → 质量矩阵
* `F_func(q, qd, t)` → 力向量
* `J_func(q)` → 约束 Jacobian
* `Jd_qd_func(q, qd)` → $\dot{J}\dot{q}$ 项
* `f_func(q)` → 约束方程值
* `f_dot_func(q, qd)` → $\dot{f}_c = J qd$ 约束速度

### 子步骤 4.3: Baumgarte 数值组装 (Baumgarte Assembly)
在 `solve_ivp` 的 RHS 回调函数中，每步动态组装并求解线性系统：
```
[M(q),    J(q)^T] [\ddot{q}]   [F(q, qd, t)]
[J(q),    0     ] [\lambda] = [\gamma(q, qd, t)]
```
其中 Baumgarte 修正项 $\gamma$ 为：
$$\gamma = -\dot{J}\dot{q} - 2\alpha \dot{f}_c - \beta^2 f_c$$
默认超参数为 $\alpha = \beta = 1$。增大 $\alpha$、$\beta$ 可增强约束稳定但增加人工耗散；减小则降低耗散但约束漂移增大。可在引擎初始化时按需调整。

### 子步骤 4.4: SVD 伪逆免疫 (Pseudo-Inverse Immunity)
当约束冗余导致 $J$ 矩阵秩亏时，**禁止直接求解线性系统**。改用 SVD 伪逆：
* 使用 `np.linalg.pinv` 计算增广矩阵的伪逆，或
* 对增广矩阵 $[M, J^T; J, 0]$ 使用 `np.linalg.lstsq` 求最小二乘解。

### 子步骤 4.5: 自适应刚性积分 (Adaptive Stiff Integration)
将 RHS 回调 $[\ddot{q}; \lambda]^T = \text{solve}(q, qd, t)$ 喂给 `scipy.integrate.solve_ivp`：
* **method**: `'Radau'`（首选）或 `'BDF'`
* **atol**: `1e-10`, **rtol**: `1e-10`
* **t_eval**: 使用前端传入的 `time_step` 生成等距输出点（仅用于渲染插值，不影响积分步长）
* 注意：`solve_ivp` 接收的是经过 Step 1.5 投影后的 $[q_0, \dot{q}_0]$，输出离散时间序列轨迹矩阵。