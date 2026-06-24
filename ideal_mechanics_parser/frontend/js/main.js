import { EventBus } from './core/EventBus.js';
import { StateMachine } from './core/StateMachine.js';
import { CommandHistory } from './core/CommandHistory.js';
import { Camera } from './canvas/Camera.js';
import { Renderer } from './canvas/Renderer.js';
import { InputHandler } from './canvas/InputHandler.js';
import { GraphBuilder } from './physics/GraphBuilder.js';
import { Validator } from './physics/Validator.js';
import { ApiClient } from './network/ApiClient.js';
import { Toolbar } from './ui/Toolbar.js';
import { Console } from './ui/Console.js';
import { PropertiesPanel } from './ui/PropertiesPanel.js';
import { ErrorToast } from './ui/ErrorToast.js';
import { uid } from './core/utils.js';

const eb = new EventBus();
const sm = new StateMachine();
const history = new CommandHistory();
const camera = new Camera();

const canvas = document.getElementById('canvas');
const renderer = new Renderer(canvas, camera, sm);
new InputHandler(canvas, camera, sm, eb);
const toolbar = new Toolbar(document.getElementById('toolbar'), eb);
new Console(document.getElementById('console'), eb, sm);
new PropertiesPanel(document.getElementById('properties-panel'), eb, sm);
new ErrorToast(eb);

const graphBuilder = new GraphBuilder(sm);
const validator = new Validator(sm);
const apiClient = new ApiClient();

const statusEl = document.getElementById('status-text');

function setStatus(text) {
  if (statusEl) statusEl.innerHTML = text;
}

eb.on('CONSOLE_TOGGLED', () => {
  camera.offsetX = camera._targetOffsetX;
  camera.offsetY = camera._targetOffsetY;
  camera.zoom = camera._targetZoom;
});

renderer.start();

eb.on('TOOL_SET', ({ tool }) => {
  sm.toolMode = tool;
  if (tool === 'add_edge') setStatus('Click a <span class="highlight">node</span>, then click another to connect');
  else if (tool === 'add_node') setStatus('Click canvas to place a <span class="highlight">MassPoint</span>');
  else if (tool === 'add_anchor') setStatus('Click canvas to place an <span class="highlight">Anchor</span>');
  else if (tool === 'select') setStatus('<span class="highlight">⊡ Anchor</span> · <span class="highlight">⊕ Point</span> · <span class="highlight">╳ Edge</span> 连线 · <span class="highlight">▶ Run</span>');
  else if (tool === 'delete') setStatus('Click a node or edge to <span class="highlight">delete</span>');
});

eb.on('SELECT', ({ id, isEdge, add }) => {
  if (!add) sm.selectedEntityIds.clear();
  sm.selectedEntityIds.add(id);
  if (isEdge) setStatus('Edge selected — edit in right panel');
  else setStatus('Node selected — drag to move, edit in right panel');
});

eb.on('DESELECT_ALL', () => {
  sm.selectedEntityIds.clear();
  setStatus('<span class="highlight">⊡ Anchor</span> · <span class="highlight">⊕ Point</span> · <span class="highlight">╳ Edge</span> 连线 · <span class="highlight">▶ Run</span>');
});

eb.on('HOVER', ({ id }) => { sm.hoveredEntityId = id; });

eb.on('ADD_ENTITY', (data) => {
  const { type, x, y } = data;
  const id = uid('n');
  const entity = { id, type, x, y, vx: data.vx || 0, vy: data.vy || 0, params: { m: 1.0, radius: 0.1 } };
  if (type === 'RigidBody') {
    entity.theta = data.theta || 0;
    entity.omega = data.omega || 0;
    entity.params = data.params || { m: 1.0, I: 0.167, shape: 'rect', length: 2, width: 0.5 };
  }
  sm.addEntity(entity);
  history.push({ undo: () => sm.removeEntity(id), redo: () => sm.addEntity({ ...entity }) });
  eb.emit('SELECT', { id, isEdge: false });
});

eb.on('DRAG_ENTITY', ({ id, dx, dy }) => {
  const ent = sm.getEntity(id);
  if (ent) { ent.x += dx; ent.y += dy; }
});

eb.on('CMD_ENTITY_MODIFY', ({ id, key, value }) => {
  const ent = sm.getEntity(id);
  if (!ent) return;
  const old = ent[key] ?? ent.params?.[key];
  const n = parseFloat(value);
  const v = (!isNaN(n) && String(n) === value) ? n : value;
  key in ent ? ent[key] = v : ent.params[key] = v;
  history.push({ undo: () => { key in ent ? ent[key] = old : ent.params[key] = old; }, redo: () => { ent[key] = v; } });
});

eb.on('CMD_GAMEMODE_CHANGE', (m) => { sm.viewPlane = m; });

eb.on('DELETE_SELECTED', () => {
  for (const id of [...sm.selectedEntityIds]) {
    if (sm.entities.has(id)) {
      const e = sm.getEntity(id); sm.removeEntity(id);
      history.push({ undo: () => sm.addEntity({ ...e }), redo: () => sm.removeEntity(id) });
    } else if (sm.edges.has(id)) {
      const e = sm.getEdge(id); sm.removeEdge(id);
      history.push({ undo: () => sm.addEdge({ ...e }), redo: () => sm.removeEdge(id) });
    }
  }
  sm.selectedEntityIds.clear();
});

eb.on('EDGE_END', ({ fromId, toId }) => {
  const modal = document.getElementById('edge-type-modal');
  const sel = document.getElementById('edge-type-select');
  sel.value = 'IdealRod';
  modal.classList.remove('hidden');

  const onConfirm = () => {
    const t = sel.value, id = uid('e'), p = {};
    if (t === 'IdealRod') p.length = 5;
    else if (t === 'IdealSpring') { p.k = 100; p.l0 = 1; }
    else if (t === 'SmoothRail') p.expr = 'y - x**2';
    else if (t === 'FixedCoordinate') { p.coord = 'x'; p.value = 0; }
    else if (t === 'LinearRelation') { p.coeffs = [1, -1, -1, 1]; p.constant = 0; }
    else if (t === 'DistanceSum') { p.via_id = ''; p.length = 10; }
    else if (t === 'AngleConstraint') p.angle = 1.5708;
    else if (t === 'HingeJoint') { p.pivot = [0, 0]; p.world = [0, 0]; }
    const edge = { id, type: t, from: fromId, to: toId, params: p };
    sm.addEdge(edge);
    history.push({ undo: () => sm.removeEdge(id), redo: () => sm.addEdge({ ...edge }) });
    eb.emit('SELECT', { id, isEdge: true });
    eb.emit('CONSOLE_LOG', { level: 'success', text: `Edge ${id} (${t})` });
    cleanup();
  };
  const onCancel = () => cleanup();
  const cleanup = () => {
    modal.classList.add('hidden');
    document.getElementById('edge-confirm').removeEventListener('click', onConfirm);
    document.getElementById('edge-cancel').removeEventListener('click', onCancel);
  };
  document.getElementById('edge-confirm').addEventListener('click', onConfirm);
  document.getElementById('edge-cancel').addEventListener('click', onCancel);
});

// --- Streaming + Playback ---

function _topologyHasEvents(topology) {
  for (const n of topology.nodes) {
    if (n.params?.radius > 0) return true;
  }
  for (const e of topology.edges) {
    if (e.type === 'SoftRope') return true;
  }
  return false;
}

function _startPlayback() {
  if (sm._playTimer) return;
  sm._playStart = performance.now();
  const tick = () => {
    if (sm.mode !== 'simulation') { sm._playTimer = null; return; }
    if (sm.isPlaying) {
      const elapsed = (performance.now() - sm._playStart - sm._pausedDuration) / 1000;
      const latest = sm.trajectory.t[sm.trajectory.t.length - 1];
      sm.playhead = Math.min(elapsed, latest);
      setStatus(`<span class="highlight">▶</span> t = ${sm.playhead.toFixed(2)}s / ${latest.toFixed(1)}s`);
    } else {
      setStatus(`<span class="highlight">⏸</span> t = ${sm.playhead.toFixed(2)}s / ${sm.trajectory.t[sm.trajectory.t.length-1].toFixed(1)}s`);
    }
    sm._playTimer = requestAnimationFrame(tick);
  };
  sm._playTimer = requestAnimationFrame(tick);
}

function _emitSimState() {
  if (sm.mode !== 'simulation') { eb.emit('SIM_STATE', 'stopped'); return; }
  eb.emit('SIM_STATE', sm.isPlaying ? 'playing' : 'paused');
}

eb.on('CMD_SIMULATION_START_GUI', () => {
  if (sm.mode === 'simulation') {
    sm.isPlaying = !sm.isPlaying;
    if (sm.isPlaying) {
      sm._pausedDuration += performance.now() - sm._pausedAt;
      setStatus(`<span class="highlight">▶</span> t = ${sm.playhead.toFixed(2)}s`);
    } else {
      sm._pausedAt = performance.now();
      setStatus(`<span class="highlight">⏸</span> t = ${sm.playhead.toFixed(2)}s`);
    }
    _emitSimState();
    return;
  }

  const errors = validator.validate();
  if (errors.length > 0) {
    for (const e of errors) eb.emit('CONSOLE_LOG', { level: 'error', text: e });
    setStatus('Validation <span class="highlight">failed</span> — check console');
    return;
  }

  const topology = graphBuilder.build();
  sm.mode = 'simulation';
  sm.playhead = 0;
  sm.isPlaying = true;
  sm.trajectory = null;
  sm._pausedDuration = 0;
  sm._pausedAt = 0;
  _emitSimState();

  const hasEvents = _topologyHasEvents(topology);
  console.log('hasEvents:', hasEvents, 'radii:', topology.nodes.map(n=>n.params?.radius));
  if (hasEvents) {
    setStatus('Computing <span class="highlight">collision</span>...');
    apiClient.solve(topology).then((data) => {
      if (sm.mode !== 'simulation') return;
      sm.trajectory = data;
      console.log("TRAJ", data.t?.length, "frames, first vx:", data.qd?.[0]?.[0], "last vx:", data.qd?.[data.qd.length-1]?.[0], "last 3 qd rows:", data.qd?.slice(-3));
      setStatus(`<span class="highlight">▶</span> t = 0.0s / ${(data.t[data.t.length-1] || 0).toFixed(1)}s`);
      _startPlayback(data);
    }).catch((err) => {
      sm.mode = 'edit';
      eb.emit('CONSOLE_LOG', { level: 'error', text: `Backend: ${err.message}` });
      setStatus('Error — check console');
      _emitSimState();
    });
    return;
  }

  // Streaming mode (no events)
  const allT = [], allQ = [];

  setStatus('Connecting to backend...');

  apiClient.streamSolve(topology, (chunk) => {
    if (chunk.error) {
      sm.mode = 'edit';
      eb.emit('ERROR', { message: chunk.error });
      eb.emit('CONSOLE_LOG', { level: 'error', text: `Backend: ${chunk.error}` });
      setStatus('Error — check console');
      _emitSimState();
      return;
    }
    if (chunk.complete) return;
    allT.push(...chunk.t);
    allQ.push(...chunk.q);
    sm.trajectory = { t: allT, q: allQ, node_order: chunk.node_order };

    if (!sm._playTimer) _startPlayback();
  }).catch((err) => {
    if (sm.mode !== 'simulation') return;
    sm.mode = 'edit';
    eb.emit('ERROR', { message: err.message });
    eb.emit('CONSOLE_LOG', { level: 'error', text: `Connection error: ${err.message}` });
    setStatus('Connection failed');
    _emitSimState();
  });
});

eb.on('CMD_SIMULATION_STOP', () => {
  sm.mode = 'edit';
  sm.playhead = 0;
  sm.trajectory = null;
  sm.isPlaying = true;
  sm._pausedDuration = 0;
  if (sm._playTimer) { cancelAnimationFrame(sm._playTimer); sm._playTimer = null; }
  setStatus('<span class="highlight">⊡ Anchor</span> · <span class="highlight">⊕ Point</span> · <span class="highlight">╳ Edge</span> 连线 · <span class="highlight">▶ Run</span>');
  _emitSimState();
});

eb.on('CMD_CANVAS_CLEAR', () => {
  const oldE = new Map(sm.entities), oldEg = new Map(sm.edges);
  sm.entities.clear(); sm.edges.clear(); sm.selectedEntityIds.clear();
  history.push({ undo: () => { for (const [i,e] of oldE) sm.entities.set(i,e); for (const [i,e] of oldEg) sm.edges.set(i,e); }, redo: () => { sm.entities.clear(); sm.edges.clear(); } });
});

eb.on('CMD_UNDO', () => history.undo());
eb.on('CMD_REDO', () => history.redo());

document.addEventListener('keydown', (e) => {
  if (e.target?.tagName === 'INPUT' || e.target?.tagName === 'TEXTAREA') return;
  if (e.ctrlKey && e.key === 'z') { e.preventDefault(); eb.emit(e.shiftKey ? 'CMD_REDO' : 'CMD_UNDO'); return; }
  if (e.key === 'Delete' || e.key === 'Backspace') { if (sm.selectedEntityIds.size > 0) eb.emit('DELETE_SELECTED'); return; }

  if (sm.mode === 'simulation') {
    if (e.key === ' ') { e.preventDefault(); eb.emit('CMD_SIMULATION_START_GUI'); return; }
    if (e.key === 'ArrowRight') {
      e.preventDefault();
      if (sm.trajectory) {
        sm.playhead = Math.min(sm.playhead + 0.1, sm.trajectory.t[sm.trajectory.t.length - 1]);
        setStatus(`<span class="highlight">⏸</span> t = ${sm.playhead.toFixed(2)}s`);
      }
    }
    if (e.key === 'ArrowLeft') {
      e.preventDefault();
      sm.playhead = Math.max(sm.playhead - 0.1, 0);
      setStatus(`<span class="highlight">⏸</span> t = ${sm.playhead.toFixed(2)}s`);
    }
  }
});
