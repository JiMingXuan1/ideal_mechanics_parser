# 后稳定投影法 (Post-Stabilization Projection)

## 问题

Baumgarte 稳定化 $\ddot{f}_c + 2\alpha\dot{f}_c + \beta^2 f_c = 0$ 是一种**人工力法**：
它在约束方程中注入人为的弹簧-阻尼修正项。无论 $\alpha、\beta$ 取多小，都会对系统总能量做功，导致长期积分下的能量漂移（测试显示 50s 双摆 ~0.6%）。

后稳定投影法是一种不引入人工力的替代方案。

## 原理

在每个积分步（或每 N 步）结束后，将状态向量 $(q, \dot{q})$ 直接投影回约束流形：

1. **位置投影**：用 Newton-Raphson 将 $q$ 投影到 $f_c(q) = 0$（复用 `core/projection.py` 的逻辑）
2. **速度投影**：将 $\dot{q}$ 投影到切空间 $J(q)\dot{q} = 0$：

   $$\dot{q} \leftarrow \dot{q} - J(q)^+ \big( J(q) \dot{q} \big)$$

   其中 $J^+$ 是 Moore-Penrose 伪逆。这等价于移除速度中垂直于约束流形的分量。

两步都不改变系统的总能量（投影是正交的），也不改变广义动量在切空间中的分量。

## 与 Baumgarte 的对比

| 特性 | Baumgarte (α=β=1) | 后稳定投影 |
|------|-------------------|-----------|
| 约束误差 | ~1e-5 稳态 | ~1e-12（机器精度） |
| 能量漂移 (50s 双摆) | ~0.6% | < 1e-10（仅受解算器截断误差） |
| 计算开销 | 无额外开销 | 每步一次 N-R 迭代 + pinv |
| 实现复杂度 | 已在 RHS 中 | 需在外层循环中插入 |

## 实现方案

### 方案 A：分段积分 + 中间投影（推荐，改动最小）

在外层 while 循环中，将 $[0, T]$ 切为多个子区间（如每 0.1s 一段），每段走完一次 `solve_ivp` 后执行投影，然后以投影后的状态启动下一段。

```
q0, qd0 = 初始值（已投影）
t_remaining = [0, T]
while t_remaining 未结束:
    t_seg = [t_current, min(t_current + 0.1, T)]
    result = solve_ivp(rhs, t_seg, state0, ...)
    q, qd = result.y
    q = project_position(q)      # N-R: q ← q - J⁺ f(q)
    qd = project_velocity(q, qd) # 切空间: qd ← qd - J⁺ (J @ qd)
    state0 = [q, qd]
    t_current = t_seg[1]
```

### 方案 B：RHS 内嵌投影（更激进）

在 `NumericalIntegrator.rhs()` 末尾，直接在计算出的 $q_{dd}$ 上做切空间投影。但下个时间步的 $q,\dot{q}$ 完全由 Radau 的自适应步长控制，投影效果受限于解算器的时间离散。

不推荐方案 B，因为 Radau 的自适应步长假设 RHS 是光滑的，投影会引入不连续性。

## 需要改动的文件

- `core/numerical.py` — 新增 `post_stabilize(q, qd)` 类方法
- `core/engine.py` — `_step5_integrate()` 中改为分段积分循环
- （可选）`system_env` 中增加 `"stabilization": "baumgarte" | "projection"` 开关

## 验证方法

- 双摆测试应显示能量漂移 < 1e-10（而非 Baumgarte 的 0.6%）
- 约束漂移测试应显示杆长误差 < 1e-12（而非 Baumgarte 的 1e-5）
