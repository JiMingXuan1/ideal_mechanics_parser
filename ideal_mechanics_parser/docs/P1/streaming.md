# 流式计算与播放控制方案

解决长仿真等待 + 无播放控制的问题。

## 1. 流式积分 (Streaming Integration)

### 架构

```
solve_ivp(0~T) → 一次性返回
          ↓
分段 solve_ivp(0~0.5, 0.5~1.0, ...) → 每段算完立即推给前端
```

后端用 SSE (Server-Sent Events) 协议，每段积分完成后 flush 一个 JSON chunk。

### 后端改动

**`core/engine.py`** 新增 `run_stream()` 方法：

```python
def run_stream(self, on_chunk, seg_duration=0.5):
    dt = float(self.topology["system_env"].get("time_step", 0.01))
    duration = float(self.topology["system_env"].get("duration", 10.0))
    t_full = np.arange(0, duration + dt, dt)
    state = np.concatenate([self.q0_projected, self.qd0])
    seg_start = 0.0
    while seg_start < duration:
        seg_end = min(seg_start + seg_duration, duration)
        mask = (t_full >= seg_start) & (t_full <= seg_end)
        t_seg = t_full[mask]
        result = solve_ivp(self._rhs, [seg_start, seg_end], state,
                           t_eval=t_seg, method="Radau", atol=1e-10, rtol=1e-10)
        if not result.success:
            on_chunk({"error": result.message, "complete": True}); return
        state = result.y[:, -1]
        on_chunk({
            "t": result.t.tolist(),
            "q": result.y[:self.nq].T.tolist(),
            "node_order": [p.id for p in self.points],
            "complete": seg_end >= duration,
        })
        seg_start = seg_end
```

**`server.py`** 新增端点 `/solve/stream`，使用 SSE 协议逐段推送。

### 前端改动

**`ApiClient.js`** 新增 `streamSolve(onChunk)` 方法。

**`main.js`** 改造 `CMD_SIMULATION_START` 使用流式 API，`CMD_SIMULATION_START_GUI` 切换播放/暂停。

## 2. 播放控制 (Play / Pause)

### 状态

```javascript
isPlaying: bool       // 是否在推进 playhead
playStartTime: float  // 当前段开始时间（用于计算暂停补偿）
```

### 按键

| 按键 | 动作 |
|------|------|
| `Space` | 切换 播放/暂停 |
| `→` | 前进 0.1s（暂停时） |
| `←` | 后退 0.1s（暂停时） |

### UI

`▶ Run` 按钮在播放态变成 `⏸`，暂停态恢复 `▶ Resume`，停止态恢复 `▶ Run`。
底部状态栏实时显示 `▶/⏸ t = 3.2s / 10.0s`。

## 3. 改动清单

| 文件 | 改动 | 行数 |
|------|------|------|
| `core/engine.py` | 新增 `run_stream()` 方法 | +40 |
| `server.py` | 新增 `/solve/stream` SSE 端点 | +30 |
| `ApiClient.js` | 新增 `streamSolve()` 方法 | +25 |
| `StateMachine.js` | 新增 `isPlaying` 属性 | +1 |
| `main.js` | 流式组装 + 播放循环暂停 + Space/←/→ | +60 |
| `index.html` | Run 按钮状态指示 | +2 |

**总计约 +160 行**

## 4. 限制

- 单线程阻塞：一段计算完才能发下一段，无法后台提前终止（后续可升级 asyncio）
- 轨迹全量在内存中追加，超长仿真需做窗口截断
