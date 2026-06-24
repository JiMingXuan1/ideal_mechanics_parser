export class StateMachine {
  constructor() {
    this.mode = 'edit';
    this.viewPlane = 'XY';
    this.toolMode = 'select';
    this.selectedEntityIds = new Set();
    this.hoveredEntityId = null;
    this.entities = new Map();
    this.edges = new Map();
    this.playhead = 0;
    this.trajectory = null;
    this.isPlaying = true;
    this._playStartTime = 0;
    this._pausedDuration = 0;
  }

  get selectedCount() {
    return this.selectedEntityIds.size;
  }

  getEntity(id) {
    return this.entities.get(id) || null;
  }

  getEdge(id) {
    return this.edges.get(id) || null;
  }

  getAllNodeIds() {
    return [...this.entities.keys()];
  }

  addEntity(entity) {
    this.entities.set(entity.id, entity);
  }

  removeEntity(id) {
    this.entities.delete(id);
    this.selectedEntityIds.delete(id);
    for (const [eid, edge] of this.edges) {
      if (edge.from === id || edge.to === id) {
        this.edges.delete(eid);
      }
    }
  }

  addEdge(edge) {
    this.edges.set(edge.id, edge);
  }

  removeEdge(id) {
    this.edges.delete(id);
  }
}
