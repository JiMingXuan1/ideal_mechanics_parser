# LCP 平行引擎

## 背景

拉格朗日乘子法在处理摩擦、堆叠、多接触等非光滑动力学时面临根本性困难：
- 摩擦是互补条件：$v_t = 0 \rightarrow |F_t| \leq \mu F_n$，不满足光滑性假设
- 多接触面的静摩擦锁死会导致秩亏和刚性方程
- SymPy 的符号推导在此场景下无优势

## 架构

LCP 不继承 Lagrange 乘子的实现，而是作为**独立平行引擎**运行：

```
用户 JSON → 拓扑解析
              ↓
       ┌──────┴──────┐
       │ 模式选择     │
       └──────┬──────┘
      Lagrange │    LCP
      ←←←←←←←┼→→→→→→→
      默认模式 │    "启用高级摩擦/堆叠" 模式
              │
         SymPy 推导   直接构建接触互补条件
         + Lagrange   使用 PGS 迭代求解
         + Radau
```

## 核心数据结构

```
LCPProblem:
  - A: 接触矩阵 (n_c × n_c)
  - b: 右侧向量 (n_c,)
  - lo, hi: 边界约束 (n_c,)
  - x: 待求解变量（法线/切向力）

PGS 求解器:
  for iter in max_iter:
    for i in range(n_c):
      x[i] = (b[i] - Σ A[i,j] * x[j]) / A[i,i]
      x[i] = clamp(x[i], lo[i], hi[i])
    if ||x - x_old|| < tol: break
```

## 切换机制

用户在前端勾选"启用高级摩擦/堆叠"后，JSON 中包含：
```json
{
  "system_env": {
    "solver": "lcp",
    "friction_coefficient": 0.3
  }
}
```

引擎检测到 `"solver": "lcp"` 后直接进入 LCP 分支，跳过 SymPy 的 Lagrange 推导。两种引擎在后端并存，由配置路由。
