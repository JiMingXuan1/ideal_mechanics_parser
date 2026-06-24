import { dist, pointToSegmentDist } from '../core/utils.js';

export class InputHandler {
  constructor(canvas, camera, stateMachine, eventBus) {
    this.canvas = canvas;
    this.camera = camera;
    this.sm = stateMachine;
    this.eb = eventBus;
    this._isDragging = false;
    this._isPanning = false;
    this._dragEntityId = null;
    this._lastMouse = { x: 0, y: 0 };
    this._sourceEdgeNode = null;

    canvas.addEventListener('mousedown', (e) => this._onMouseDown(e));
    canvas.addEventListener('mousemove', (e) => this._onMouseMove(e));
    canvas.addEventListener('mouseup', (e) => this._onMouseUp(e));
    canvas.addEventListener('wheel', (e) => this._onWheel(e), { passive: false });
    canvas.addEventListener('contextmenu', (e) => e.preventDefault());
  }

  _screenToWorld(sx, sy) {
    return this.camera.screenToWorld(sx, sy);
  }

  _hitTestNode(sx, sy) {
    for (const [id, entity] of this.sm.entities) {
      const s = this.camera.worldToScreen(entity.x, entity.y);
      if (dist(sx, sy, s.x, s.y) < 12) return { type: 'node', id, entity };
    }
    return null;
  }

  _hitTestEdge(sx, sy) {
    for (const [id, edge] of this.sm.edges) {
      const fromEnt = this.sm.getEntity(edge.from);
      const toEnt = edge.to && this.sm.getEntity(edge.to);
      if (!fromEnt) continue;
      const p1 = this.camera.worldToScreen(fromEnt.x, fromEnt.y);
      if (toEnt) {
        const p2 = this.camera.worldToScreen(toEnt.x, toEnt.y);
        if (pointToSegmentDist(sx, sy, p1.x, p1.y, p2.x, p2.y) < 6)
          return { type: 'edge', id, edge };
      } else {
        if (dist(sx, sy, p1.x, p1.y) < 12)
          return { type: 'edge', id, edge };
      }
    }
    return null;
  }

  _tryStartEdgeMode(nodeId) {
    if (this._sourceEdgeNode === null) {
      this._sourceEdgeNode = nodeId;
      this.eb.emit('CONSOLE_LOG', {
        level: 'info',
        text: `Click target node to connect from ${nodeId}`
      });
      return true;
    }
    return false;
  }

  _tryFinishEdge(targetId) {
    if (this._sourceEdgeNode !== null && targetId !== this._sourceEdgeNode) {
      this.eb.emit('EDGE_END', { fromId: this._sourceEdgeNode, toId: targetId });
      this._sourceEdgeNode = null;
      return true;
    }
    this._sourceEdgeNode = null;
    return false;
  }

  _onMouseDown(e) {
    this._lastMouse.x = e.clientX;
    this._lastMouse.y = e.clientY;

    const nodeHit = this._hitTestNode(e.clientX, e.clientY);
    const edgeHit = nodeHit ? null : this._hitTestEdge(e.clientX, e.clientY);

    const wantsEdge = e.shiftKey || this.sm.toolMode === 'add_edge';

    if (wantsEdge && nodeHit) {
      if (this._sourceEdgeNode === null) {
        this._sourceEdgeNode = nodeHit.id;
        this.eb.emit('CONSOLE_LOG', {
          level: 'info',
          text: `Click target node to connect from ${nodeHit.id}`
        });
      } else if (nodeHit.id !== this._sourceEdgeNode) {
        this.eb.emit('EDGE_END', { fromId: this._sourceEdgeNode, toId: nodeHit.id });
        this._sourceEdgeNode = null;
      } else {
        this._sourceEdgeNode = null;
        this.eb.emit('CONSOLE_LOG', { level: 'error', text: 'Edge cancelled' });
      }
      return;
    }

    if (this.sm.toolMode === 'add_node') {
      const world = this._screenToWorld(e.clientX, e.clientY);
      this.eb.emit('ADD_ENTITY', { type: 'MassPoint', x: world.x, y: world.y });
      return;
    }

    if (this.sm.toolMode === 'add_rigidbody') {
      const world = this._screenToWorld(e.clientX, e.clientY);
      this.eb.emit('ADD_ENTITY', {
        type: 'RigidBody', x: world.x, y: world.y,
        theta: 0, vx: 0, vy: 0, omega: 0,
        params: { m: 1.0, I: 0.167, shape: 'rect', length: 2, width: 0.5 },
      });
      return;
    }

    if (this.sm.toolMode === 'add_anchor') {
      const world = this._screenToWorld(e.clientX, e.clientY);
      this.eb.emit('ADD_ENTITY', { type: 'Anchor', x: world.x, y: world.y });
      return;
    }

    if (nodeHit) {
      this._isDragging = true;
      this._dragEntityId = nodeHit.id;
      this.eb.emit('SELECT', { id: nodeHit.id, isEdge: false, add: e.ctrlKey });
    } else if (edgeHit) {
      this.eb.emit('SELECT', { id: edgeHit.id, isEdge: true, add: false });
    } else {
      this.eb.emit('DESELECT_ALL');
      this._isPanning = true;
    }
  }

  _onMouseMove(e) {
    const dx = e.clientX - this._lastMouse.x;
    const dy = e.clientY - this._lastMouse.y;
    this._lastMouse.x = e.clientX;
    this._lastMouse.y = e.clientY;

    const nodeHit = this._hitTestNode(e.clientX, e.clientY);
    const edgeHit = nodeHit ? null : this._hitTestEdge(e.clientX, e.clientY);
    this.eb.emit('HOVER', { id: nodeHit ? nodeHit.id : (edgeHit ? edgeHit.id : null) });

    if (this._isDragging && this._dragEntityId) {
      const wd = { x: dx / this.camera.zoom, y: -dy / this.camera.zoom };
      this.eb.emit('DRAG_ENTITY', { id: this._dragEntityId, dx: wd.x, dy: wd.y });
    }
    if (this._isPanning) this.camera.pan(dx, dy);
  }

  _onMouseUp(e) {
    if (this._isDragging || this._isPanning) {
      this._isDragging = false;
      this._isPanning = false;
      this._dragEntityId = null;
    }
  }

  _onWheel(e) {
    e.preventDefault();
    const world = this._screenToWorld(e.clientX, e.clientY);
    const factor = e.deltaY > 0 ? 0.9 : 1.1;
    this.camera.zoomAt(world.x, world.y, factor);
  }
}
