# 求解器参数外部可配置

## 问题

目前 Baumgarte $\alpha、\beta$ 和积分器 `atol、rtol` 是硬编码在 `numerical.py` 类属性中的。用户无法针对特定系统调参。

## 改动

在 JSON `system_env` 中增加可选字段：

```json
{
  "system_env": {
    "view_plane": "XY",
    "gravity": 9.81,
    "time_step": 0.01,
    "duration": 10.0,
    "baumgarte_alpha": 1.0,
    "baumgarte_beta": 1.0,
    "atol": 1e-10,
    "rtol": 1e-10
  }
}
```

引擎解析逻辑：优先使用 JSON 传入值，缺失时回退为 `numerical.py` 中的类属性默认值。

## 需要改动的文件

- `core/numerical.py` — `__init__` 增加 `alpha, beta, atol, rtol` 参数
- `core/engine.py` — `_step5_integrate()` 从 `system_env` 读取并传递给 `NumericalIntegrator`
- `io_handler/parser.py` — 非必需，缺少时用默认值即可
