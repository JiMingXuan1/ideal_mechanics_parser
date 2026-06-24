# 事件驱动框架 (Event-Driven Framework)

## 问题

Phase 1 的积分过程是连续的：一次 `solve_ivp` 调用覆盖整个 `[0, duration]` 区间。Phase 2 碰撞和软绳要求在**特定时刻打断积分**、**修改系统状态**、**然后重启积分**。这个"中断-突变-重启"范式需要一个统一的事件驱动框架。

## 架构总览

```
用户输入 ──→ Engine.run_events()
                  │
                  ▼
           ┌──────────────┐
           │  Step1~4     │  构建符号方程组（无事件约束版本）
           │  (同 Phase1) │
           └──────┬───────┘
                  ▼
           ┌──────────────┐
           │ solve_ivp    │  events=[e1, e2, ...]
           │ (带事件监控)  │  terminal=True
           └──────┬───────┘
                  │ t_event 触发
                  ▼
           ┌──────────────┐
           │ on_event()   │  事件回调: 提取q/qd，应用突变
           └──────┬───────┘
                  │ 修改后的 state
                  ▼
           ┌──────────────┐
           │ solve_ivp    │  以新状态 + 新拓扑重启
           │ (重新开始)    │  (或继续积分, 拓扑不变)
           └──────┬───────┘
                  │ ...
                  ▼
               完成 / 下一事件
```

## 事件定义

每个事件是一个字典：

```python
{
    "name": "collision_m1_m2",
    "func": lambda t, state: dist - (r1 + r2),  # 零交叉函数
    "terminal": True,                             # 触发后终止积分
    "direction": -1,                              # 仅监控由正变负
    "handler": collision_handler,                 # 回调函数
}
```

### 零交叉函数签名

```python
def event_func(t: float, state: np.ndarray) -> float:
    ...
```

与 SciPy `solve_ivp(events=...)` 的要求完全一致。

### 事件注册

事件由上层模块（如 `CollisionDetector`、`SoftRopeStateMachine`）向 Engine 注册：

```python
class CollisionDetector:
    def register_events(self, edges, points, sm):
        events = []
        for each collision pair (i, j):
            e = {
                "name": f"collision_{i}_{j}",
                "func": self._make_event(i, j, r_i, r_j, sm),
                "terminal": True,
                "direction": -1,
                "handler": self._handle_collision,
            }
            events.append(e)
        return events
```

## 突变处理

事件触发后，handler 接收 `(state, event_info)` 并返回新状态：

```python
def collision_handler(state, event_info):
    """
    Args:
        state: np.ndarray, 触发时刻的 [q; qd]
        event_info: {"name", "t_event", "indices", "extra"}
    Returns:
        new_state: np.ndarray (修改速度后的 [q; qd])
        topology_change: None 或 新的拓扑
    """
    ...
    return new_state, topology_change
```

### 两种情况

**A. 拓扑不变（碰撞）**：仅修改速度（冲量跳变），拓扑不变。直接以新状态继续积分。

**B. 拓扑改变（软绳绷紧/松弛）**：修改速度 + 增删约束方程。必须重新执行 Step 1~4 构建全新符号方程组，然后重启。此时 engine 需要完全重建 `LagrangesMethod` 和 `NumericalIntegrator`。

## Engine.run_events() 接口

```python
def run_events(self, on_chunk):
    self._step1_instantiate()
    self._step2_project()
    self._step3_energy()
    self._step4_constraints()

    events = self._register_events()  # 收集所有事件的零交叉函数
    state = np.concatenate([self.q0_projected, self.qd0])
    t_current = 0.0
    duration = self._duration
    max_mutations = self._max_mutations or 100
    mutation_count = 0

    while t_current < duration and mutation_count < max_mutations:
        result = solve_ivp(
            self.rhs, [t_current, duration], state,
            events=events,
            method="Radau", atol=1e-10, rtol=1e-10,
            t_eval=None,  # 不指定 t_eval，由事件精确打断
        )

        if hasattr(result, 't_events') and any(len(te) > 0 for te in result.t_events):
            # 事件触发
            t_event = result.t_events[0][0]
            state_event = result.y_events[0][0]

            handler = event_handlers[0]
            new_state, topology_change = handler(state_event)

            if topology_change is not None:
                self._rebuild_topology(topology_change)
                self._step1_instantiate()
                self._step2_project()   # 只投影新状态的 q
                self._step3_energy()
                self._step4_constraints()
                events = self._register_events()
                state = new_state
            else:
                state = new_state

            mutation_count += 1
            on_chunk({
                "t_event": t_event,
                "event": events[0]["name"],
                "state": state.tolist(),
                "event_type": "collision" if topology_change is None else "mutation",
            })
            t_current = t_event
        else:
            # 无事件，正常结束或完成最后一段
            on_chunk({
                "t": result.t.tolist(),
                "q": result.y[:self.sm.nq].T.tolist(),
                "complete": True,
            })
            break

    if mutation_count >= max_mutations:
        on_chunk({"error": "Max mutations exceeded", "complete": True})
```

## Streaming 兼容方案

Streaming 模式下（`POST /solve/stream`），每 0.5s 的定长时间窗口与事件打断存在冲突。解决方案：

### 方案 A：双模式入口

Engine 提供两个入口：

| 入口 | 用途 | Streaming 兼容 |
|------|------|---------------|
| `run()` | 无事件场景（纯质点/纯约束） | ✅ 当前方式 |
| `run_events()` | 有事件场景（碰撞/软绳） | ❌ 不支持 SSE streaming |

前端根据拓扑中是否包含碰撞/软绳实体，自动选择端点：

- 无事件拓扑 → `POST /solve/stream`（SSE 流式）
- 有事件拓扑 → `POST /solve`（批量等待，一次性返回所有帧）

这样 `run_events()` 不需要兼容 streaming 的分段约束。用户看到"计算中..."等待条。

### 方案 B：事件段封装（未来优化）

在 `run_events()` 中，两事件之间的连续段仍然可以走 streaming：

```
[0.0 ─── event_at_1.23 ─── 3.45 ─── event_at_7.89 ─── 10.0]
  ├─ stream chunk 1 ─┤         ├─ stream chunk 2 ─┤
       (0.0→1.23)               (1.23→3.45)
```

每段以 `solve_ivp(events=...)` 驱动，到达事件时发送特殊 `event` 消息，然后继续下一段。但 SSE 客户端需要处理乱序到达的帧。

**Phase 2 采用方案 A**。方案 B 留给 Phase 3 优化。

## 事件队列

当多个事件在极短时间内同时到达（如三体同时碰撞），竞争条件处理：

```
1. solve_ivp 终止返回 → 检查所有 t_events
2. 选择最早触发的事件（min t_event）
3. 处理该事件 → 更新状态
4. 立即重新检查新状态是否触发其他事件（迭代，最多 5 次）
5. 若 5 次内稳定 → 继续积分
6. 若 5 次仍振荡 → 抛出 TopologyError
```

## 测试策略

| 测试 | 场景 | 验证 |
|------|------|------|
| 单次碰撞 | 质点以初速度冲向固定质点 | 速度方向反转，能量守恒（e=1） |
| 非弹性碰撞 | 同上，e=0.5 | 速度变化符合恢复系数 |
| 软绳绷紧 | 两质点分离至绳长极限 | 约束瞬间激活，能量损耗正确 |
| 软绳松弛 | 绷紧后反向运动 | 约束解除 |
| 多事件序列 | 碰撞后立即触发软绳事件 | 事件队列正确序列化 |
| 最大突变次数 | 高频振荡场景 | 触发护栏并报错 |
