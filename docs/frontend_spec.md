# 前端架构规范 (Frontend Architecture Spec)

本规范定义了 Ideal Mechanics Parser 的前端原生 JS 架构标准。系统的核心设计哲学为 **"极简依赖、原生渲染、基于 EventBus 的极度解耦"**。任何前端模块均不允许直接操作其他模块的内部状态。

---

## 1. 技术栈限制 (Tech Stack Constraints)

- **核心框架**：绝对禁止引入 React、Vue、Angular 等虚拟 DOM 框架，禁止引入 Three.js 等重量级引擎。
- **语言与模块化**：严格使用 Vanilla JS (ES6 Modules)。所有核心逻辑必须使用面向对象 (Class) 封装。
- **渲染层**：纯原生 HTML5 Canvas 2D API 配合 `requestAnimationFrame`。
- **样式层**：极简原生 CSS，确保画布全屏，UI 控件使用绝对定位层叠于画布之上。

---

## 2. 目录结构蓝图 (Directory Blueprint)

```text
frontend/
├── index.html               # 唯一的 HTML 入口，包含画布与控制台 DOM 骨架
├── css/
│   ├── main.css             # 全局布局样式（重置样式、全屏画布等）
│   └── console.css          # 控制台滑出动画与极客主题样式
└── js/
    ├── main.js              # 程序主入口：实例化所有模块并进行依赖注入
    ├── core/
    │   ├── EventBus.js      # 【核心枢纽】发布/订阅模式实现，解耦所有模块
    │   ├── StateMachine.js  # 全局状态源（管理 view_plane、播放状态等）
    │   └── CommandHistory.js# 撤销/重做栈
    ├── canvas/
    │   ├── Renderer.js      # 渲染引擎：监听状态变化并执行帧重绘
    │   ├── InputHandler.js  # 交互捕获：处理鼠标点按、拖拽、滚轮缩放与坐标转换
    │   ├── Camera.js        # 坐标系变换 + 缩放/平移状态
    │   └── Entities.js      # 图元绘制类（质点、弹簧、轨道的 Canvas 绘制逻辑）
    ├── physics/
    │   ├── GraphBuilder.js  # 拓扑打包：将图元状态序列化为标准的 JSON 格式
    │   └── Validator.js     # 防呆预检：发送给后端前的拓扑合法性检验
    ├── network/
    │   └── ApiClient.js     # 通信层：封装 Fetch，向 Python 发送拓扑并接收时间序列
    └── ui/
        ├── Toolbar.js       # 左侧拖拽图元的工具栏逻辑
        ├── Console.js       # 隐藏控制台逻辑，负责指令捕获与解析
        ├── PropertiesPanel.js# 选中实体的属性编辑面板
        └── ErrorToast.js    # 非侵入式错误提示
```

---

## 3. 核心枢纽：EventBus 解耦规范

禁止跨文件直接调用实例方法（例如禁止 Console 直接调用 Renderer.draw()）。一切跨模块交互必须通过 EventBus 广播。

### 标准事件流示例

```
Console 监听到用户输入指令，执行：eventBus.emit('CMD_GAMEMODE_CHANGE', 'XZ')
  → StateMachine 监听到该事件，更新自身的 currentPlane 变量
  → Renderer 监听到该事件，擦除竖直面重力箭头，绘制水平桌面网格
```

### 撤销/重做集成

`CommandHistory` 模块挂载在 EventBus 上，自动记录以下命令事件：

| 事件 | 描述 | 载荷 |
|------|------|------|
| `CMD_ENTITY_ADD` | 添加节点或边 | `{ entity }` |
| `CMD_ENTITY_DELETE` | 删除节点或边 | `{ entity }` |
| `CMD_ENTITY_MOVE` | 拖拽移动节点 | `{ id, from, to }` |
| `CMD_ENTITY_MODIFY` | 修改属性 | `{ id, key, oldVal, newVal }` |

每个命令事件包含 `undo()` / `redo()` 方法。Ctrl+Z 触发 `CMD_UNDO`，Ctrl+Shift+Z 触发 `CMD_REDO`。

---

## 4. 实体创建与交互流程

整个前端交互流分为四个阶段，缺一不可：

```text
创建实体
  ├── 从 Toolbar 拖拽图元图标到画布释放
  └── 图元出现在鼠标释放位置（自动分配唯一 ID）
选中实体
  ├── 单击画布空白处 → 取消全选
  ├── 单击节点 → 选中，高亮显示，PropertiesPanel 弹出
  └── 按住 Shift 单击多个 → 多选
创建连接
  ├── 选中一个节点 → 按住 Shift + 拖拽到另一节点 → 弹出边类型选择面板
  │   （选择 IdealRod / IdealSpring / SmoothRail，填入参数）
  └── 或：选中两个节点后在 PropertiesPanel 中点击"连接"按钮
编辑属性
  ├── 选中节点/边 → PropertiesPanel 显示可编辑字段
  │   （质点：m, x, y, vx, vy；弹簧：k, l0；杆：length；轨道：expr）
  └── 或：Console 指令 set mass 2、set k 100
```

### 连线补充规则

- 不允许 Anchor ↔ Anchor 连接
- 不允许连接自身（from === to）
- 不允许重复边（同一对节点之间只能有一条同类型边）
- 违反规则时 ErrorToast 弹出提示，连接不生效

---

## 5. 隐藏控制台设计 (The Hacker Console)

控制台是该系统高级用户的"逃生舱"与极客交互入口。

### 唤醒机制

在 `window` 全局监听 `keydown` 事件。当捕获到 `` ` ``（波浪号）键且当前无表单获取焦点时，从屏幕顶部下拉滑出控制台界面。再次按下隐藏。唤醒时输入框必须强制 `focus()`。

### 指令解析引擎

输入内容按空格切割解析，需支持以下核心路由：

| 指令 | 效果 | 触发事件 |
|------|------|---------|
| `gamemode xy` | 水平桌面（零重力俯视） | `CMD_GAMEMODE_CHANGE` |
| `gamemode xz` | 竖直立面（重力场，Y 代表竖直坐标） | `CMD_GAMEMODE_CHANGE` |
| `run` | 启动模拟 | `CMD_SIMULATION_START` |
| `stop` | 退回编辑模式 | `CMD_SIMULATION_STOP` |
| `clear` | 清空画布所有图元 | `CMD_CANVAS_CLEAR` |
| `set <key> <val>` | 修改当前选中的实体属性 | `CMD_ENTITY_MODIFY` |
| `undo` | 撤销上次操作 | `CMD_UNDO` |
| `redo` | 重做 | `CMD_REDO` |
| `help` | 在控制台打印指令列表 | — |

### 反馈机制

- 合法的指令执行后，在控制台历史区以白色/绿色文字输出确认信息
- 不合法的指令/参数以**红色**字体输出错误提示
- 后端返回的错误（`TopologyError`、`ProjectionError`）也以红色显示，并将系统退回编辑模式

---

## 6. 渲染器与渲染循环规范 (Render Loop)

`Renderer.js` 必须维持一个高效的 `requestAnimationFrame` 循环。

### 帧状态

| 模式 | 行为 |
|------|------|
| **编辑态** | 根据 StateMachine 中存储的静态图元坐标进行绘制，响应用户的拖拽更新 |
| **播放态** | 根据 ApiClient 获取的时间序列矩阵，利用 `playhead` 按 `time_step` 插值更新图元坐标并重绘 |

### 坐标系变换

`Camera.js` 负责维护**三层坐标转换链**：

```
屏幕坐标 (px) ←[Camera.transform]→ 世界坐标 (物理米)
                                           ↕
                                     画布逻辑坐标 (Canvas 坐标空间)
```

- `InputHandler` 将鼠标事件（px）通过 `Camera.screenToWorld()` 转为物理坐标
- `Renderer` 通过 `Camera.worldToScreen()` 将物理坐标转回 px 绘制
- 滚轮缩放：修改 `Camera.zoom`，平移：拖拽画布空白区修改 `Camera.offset`
- 缩放/平移动画带缓动 (`ease-out`)，禁止跳变

---

## 7. StateMachine 全局状态定义

`StateMachine` 是系统的**唯一数据源**，必须维护以下状态。

```js
class StateMachine {
  mode: 'edit' | 'simulation'     // 当前模式
  viewPlane: 'XY' | 'XZ'          // 投影平面
  toolMode: 'select' | 'add_node' | 'add_edge'  // 当前工具
  selectedEntityIds: Set<string>   // 选中的实体 ID
  hoveredEntityId: string | null   // 鼠标悬浮的实体
  entities: Map<string, EntityState> // 所有图元状态（单一数据源）
  edges: Map<string, EdgeState>      // 所有边状态
  playhead: number                   // 播放进度（s）
  trajectory: { t: number[], q: number[][], qd: number[][] } | null
}
```

**规则**：Renderer 和 Toolbar 等展示层模块只能**读取** `StateMachine`，绝不能直接写入。所有修改必须通过 EventBus 事件驱动。

---

## 8. ApiClient 与错误处理

`ApiClient.js` 负责与 Python 后端的单次请求-应答通信。

### 请求流程

```
1. StateMachine 收集当前 entities + edges 状态
2. GraphBuilder 构建 JSON 拓扑（含 system_env）
3. Validator 进行前端预检：
   - 检测孤岛节点（与任何边不相连的质点/锚点）
   - 检测非法拓扑闭环
   - 检测缺失必选 param 字段
4. 预检通过 → ApiClient.fetch() 发送 POST 请求
5. 预检不通过 → ErrorToast 显示预检错误，不发送请求
```

### 错误处理

| 场景 | 处理 |
|------|------|
| 网络错误（后端未启动） | Console 红色显示"连接后端失败"，退回编辑态 |
| 后端返回 HTTP 400/500 | 解析响应中的 `error` 字段（`TopologyError` 等），Console红色显示 |
| 后端超时 (>30s) | Console 红色显示"后端超时"，退回编辑态 |
| 后端返回成功 | 时间序列存入 StateMachine.trajectory，切换到播放态 |

---

## 9. PropertiesPanel 属性编辑面板

当选中一个实体时，`PropertiesPanel` 从屏幕右侧滑入，显示当前选中实体的可编辑字段。

| 实体类型 | 可编辑字段 |
|---------|-----------|
| Anchor | `init_pos` (x, y) |
| MassPoint | `m`, `init_state` (x, y, vx, vy), `x_expr`, `y_expr` |
| IdealRod | `length` |
| IdealSpring | `k`, `l0` |
| SmoothRail | `expr` |

每次字段修改触发 `CMD_ENTITY_MODIFY` 事件，由 `CommandHistory` 记录以支持撤销。

---

## 10. 数据流转与后端握手

前端必须确保用户通过 `gamemode` 设定的模式，在用户敲击 `run` 指令时，被正确封装在向后端发送的 JSON 数据头中。

```json
{
  "system_env": {
    "view_plane": "XZ",
    "gravity": 9.81,
    "time_step": 0.01,
    "duration": 10.0
  },
  "nodes": [
    {"id": "n1", "type": "Anchor", "init_pos": [0, 5]},
    {"id": "n2", "type": "MassPoint", "params": {"m": 1.0}, "init_state": {"x": 3, "y": 1, "vx": 0, "vy": 0}}
  ],
  "edges": [
    {"id": "e1", "type": "IdealRod", "from": "n1", "to": "n2", "params": {"length": 5.0}}
  ]
}
```

其中 `view_plane` 和 `gravity` 由 `StateMachine.viewPlane` 决定（`XY` 水平面模式下后端自动忽略重力，`XZ` 竖直面模式下 Y 视为竖直坐标、重力生效）。

---

## 附录：与后端文档的交叉引用

| 前端概念 | 后端对应文档 |
|---------|-------------|
| 拓扑 JSON 格式 | `p1_features.md` §2 逃生舱 |
| 运行流程 | `backend_pipeline.md` 5 步流水线 |
| 图元类型 | `core_architecture.md` §4 图元库 |
| 数值装甲 | `core_architecture.md` §3 数值装甲 |
| 错误类型 | `backend_pipeline.md` Step 1.5 的 `TopologyError` |
