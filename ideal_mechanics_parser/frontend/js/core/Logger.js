const MAX_LOG = 200;

export class Logger {
  constructor(eventBus, stateMachine) {
    this.eb = eventBus;
    this.sm = stateMachine;
    this._queue = [];
    this._id = 0;

    // Subscribe to all CMD_, SYS_, ERROR events
    const allEvents = [
      'CMD_GAMEMODE_CHANGE', 'CMD_SIMULATION_START_GUI', 'CMD_SIMULATION_STOP',
      'CMD_ENTITY_MODIFY', 'CMD_CANVAS_CLEAR', 'CMD_UNDO', 'CMD_REDO',
      'DELETE_SELECTED', 'EDGE_END', 'ADD_ENTITY', 'DRAG_ENTITY',
      'CONSOLE_LOG', 'ERROR', 'SELECT', 'DESELECT_ALL',
    ];
    for (const ev of allEvents) {
      this.eb.on(ev, (data) => this._log('event', ev, data));
    }
  }

  _log(type, name, payload) {
    this._queue.push({
      id: ++this._id,
      t: Date.now(),
      type,
      name,
      payload: this._safe(payload),
    });
    if (this._queue.length > MAX_LOG) this._queue.shift();
  }

  logNetwork(request, response) {
    this._log('network', 'API_CALL', { request: this._safe(request), response: this._safe(response) });
  }

  _safe(obj) {
    try { return JSON.parse(JSON.stringify(obj)); } catch { return String(obj); }
  }

  _snapshot() {
    const sm = this.sm;
    const ents = [];
    for (const [id, e] of sm.entities) ents.push({ id, type: e.type, x: e.x, y: e.y, vx: e.vx, vy: e.vy, params: e.params });
    const edges = [];
    for (const [id, e] of sm.edges) edges.push({ id, type: e.type, from: e.from, to: e.to, params: e.params });
    return {
      mode: sm.mode,
      viewPlane: sm.viewPlane,
      gravitationEnabled: sm.gravitationEnabled,
      trailsEnabled: sm.trailsEnabled,
      playhead: sm.playhead,
      hasTrajectory: sm.trajectory !== null,
      entities: ents,
      edges,
    };
  }

  exportDump() {
    const now = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    const dump = {
      version: '0d4774c',
      exportedAt: new Date().toISOString(),
      stateMachine: this._snapshot(),
      logs: this._queue,
    };
    const blob = new Blob([JSON.stringify(dump, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `ideal_mechanics_crash_${now}.json`;
    a.click();
    URL.revokeObjectURL(url);
    return dump;
  }
}
