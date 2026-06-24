import { Entities } from './Entities.js';

export class Renderer {
  constructor(canvas, camera, stateMachine) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.camera = camera;
    this.sm = stateMachine;
    this._animId = null;
    this._lastTime = 0;
    this._resize();
    window.addEventListener('resize', () => this._resize());
  }

  _resize() {
    const w = window.innerWidth;
    const h = window.innerHeight;
    this.canvas.width = w;
    this.canvas.height = h;
    this._width = w;
    this._height = h;
    if (!this.camera._centered) {
      this.camera.centerOn(w / 2, h / 2);
    }
  }

  start() {
    this._animId = requestAnimationFrame((t) => this._loop(t));
  }

  _loop(time) {
    this._animId = requestAnimationFrame((t) => this._loop(t));
    this._tick(time - this._lastTime);
    this._lastTime = time;
  }

  _tick(dt) {
    this.camera.update(0.12);
    this._draw();
  }

  _draw() {
    const ctx = this.ctx;
    const w = this._width;
    const h = this._height;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#f8f9fa';
    ctx.fillRect(0, 0, w, h);

    Entities.drawGrid(ctx, this.camera, w, h);

    if (this.sm.trailsEnabled && this.sm.mode === 'simulation' && this.sm.trajectory) {
      this._drawTrails();
    }

    if (this.sm.mode === 'simulation' && this.sm.trajectory) {
      this._drawSimulation();
    } else {
      this._drawEdges();
      this._drawEntities();
    }

  }

  _drawTrails() {
    const traj = this.sm.trajectory;
    if (!traj || !traj.t || traj.t.length === 0) return;
    const totalFrames = traj.t.length;
    const totalT = traj.t[totalFrames - 1];
    const currentIdx = Math.min(
      Math.floor((this.sm.playhead / totalT) * (totalFrames - 1)),
      totalFrames - 1
    );
    if (currentIdx < 1) return;

    const ctx = this.ctx;
    const nodeOrder = traj.node_order;
    const bodyDofs = traj.body_dofs || nodeOrder.map(() => 2);
    const COLORS = ['#0969da','#cf222e','#2da44e','#8250df','#d29922','#0550ae','#1a7f37','#e16f24'];

    let qi = 0;
    for (let bi = 0; bi < nodeOrder.length; bi++) {
      const dof = bodyDofs[bi];
      const color = COLORS[bi % COLORS.length];

      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.globalAlpha = 0.45;
      ctx.beginPath();
      let started = false;
      for (let f = 0; f <= currentIdx; f++) {
        const q = traj.q[f];
        if (!q) continue;
        const wx = q[qi];
        const wy = q[qi + 1];
        if (wx == null || wy == null) continue;
        const s = this.camera.worldToScreen(wx, wy);
        if (!started) {
          ctx.moveTo(s.x, s.y);
          started = true;
        } else {
          ctx.lineTo(s.x, s.y);
        }
      }
      ctx.stroke();
      qi += dof;
    }
    ctx.globalAlpha = 1;
  }

  _drawPivots() {
    const ctx = this.ctx;
    for (const [id, edge] of this.sm.edges) {
      const fromEnt = this.sm.getEntity(edge.from);
      const fromPivot = edge.params?.from_pivot;
      if (fromEnt && fromEnt.type === 'RigidBody' && fromPivot) {
        const theta = fromEnt.theta || 0;
        const ct = Math.cos(theta), st = Math.sin(theta);
        const wx = fromEnt.x + fromPivot[0] * ct - fromPivot[1] * st;
        const wy = fromEnt.y + fromPivot[0] * st + fromPivot[1] * ct;
        const s = this.camera.worldToScreen(wx, wy);
        ctx.beginPath();
        ctx.arc(s.x, s.y, 4, 0, Math.PI * 2);
        ctx.fillStyle = '#8250df';
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.stroke();
      }
      const toPivot = edge.params?.to_pivot;
      const toEnt = edge.to && this.sm.getEntity(edge.to);
      if (toEnt && toEnt.type === 'RigidBody' && toPivot) {
        const theta = toEnt.theta || 0;
        const ct = Math.cos(theta), st = Math.sin(theta);
        const wx = toEnt.x + toPivot[0] * ct - toPivot[1] * st;
        const wy = toEnt.y + toPivot[0] * st + toPivot[1] * ct;
        const s = this.camera.worldToScreen(wx, wy);
        ctx.beginPath();
        ctx.arc(s.x, s.y, 4, 0, Math.PI * 2);
        ctx.fillStyle = '#8250df';
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.stroke();
      }
    }
  }

  _drawEntities() {
    const ctx = this.ctx;
    const sel = this.sm.selectedEntityIds;
    const hover = this.sm.hoveredEntityId;

    for (const [id, entity] of this.sm.entities) {
      const s = this.camera.worldToScreen(entity.x, entity.y);
      const isHover = hover === id;
      const isSel = sel.has(id);

      if (entity.type === 'Anchor') {
        Entities.drawAnchor(ctx, s.x, s.y, isHover, isSel);
      } else if (entity.type === 'RigidBody') {
        const len = (entity.params?.length || 2);
        const wid = (entity.params?.width || 0.5);
        const shape = entity.params?.shape || 'rect';
        Entities.drawRigidBody(ctx, s.x, s.y, entity.theta || 0, isHover, isSel, id, {
          length: len * this.camera.zoom,
          width: wid * this.camera.zoom,
          shape,
        });
      } else {
        Entities.drawMassPoint(ctx, s.x, s.y, isHover, isSel, id);
      }
    }
  }

  _drawEdges() {
    const ctx = this.ctx;
    const sel = this.sm.selectedEntityIds;
    const hover = this.sm.hoveredEntityId;

    for (const [id, edge] of this.sm.edges) {
      const fromEnt = this.sm.getEntity(edge.from);
      const toEnt = edge.to && this.sm.getEntity(edge.to);
      if (!fromEnt) continue;

      const p1 = this.camera.worldToScreen(fromEnt.x, fromEnt.y);
      let p2 = null;
      if (toEnt) p2 = this.camera.worldToScreen(toEnt.x, toEnt.y);

      const isH = hover === id;
      const isS = sel.has(id);

      switch (edge.type) {
        case 'IdealRod':
          if (p2) Entities.drawIdealRod(ctx, p1.x, p1.y, p2.x, p2.y, isH, isS); break;
        case 'IdealSpring':
          if (p2) Entities.drawIdealSpring(ctx, p1.x, p1.y, p2.x, p2.y, isH, isS); break;
        case 'SmoothRail':
          if (p2) Entities.drawSmoothRail(ctx, p1.x, p1.y, p2.x, p2.y, isH, isS); break;
        case 'FixedCoordinate':
          Entities.drawFixedCoordinate(ctx, p1.x, p1.y, edge.params, isH, isS); break;
        case 'LinearRelation':
          Entities.drawLinearRelation(ctx, p1.x, p1.y, p2 ? p2.x : null, p2 ? p2.y : null, edge.params, isH, isS); break;
        case 'DistanceSum': {
          const viaEnt = edge.params?.via_id && this.sm.getEntity(edge.params.via_id);
          if (p2 && viaEnt) {
            const via = this.camera.worldToScreen(viaEnt.x, viaEnt.y);
            Entities.drawDistanceSum(ctx, p1.x, p1.y, p2.x, p2.y, via, isH, isS);
          }
          break;
        }
        case 'AngleConstraint':
          if (p2) Entities.drawAngleConstraint(ctx, p1.x, p1.y, p2.x, p2.y, edge.params?.angle || 0, isH, isS); break;
      }
    }
  }

  _drawSimulation() {
    const traj = this.sm.trajectory;
    if (!traj || traj.t.length === 0) return;

    const totalT = traj.t[traj.t.length - 1];
    const frac = Math.min(this.sm.playhead / totalT, 1);
    const idx = Math.min(Math.floor(frac * (traj.t.length - 1)), traj.t.length - 1);
    const q = traj.q[idx];
    const nodeOrder = traj.node_order;
    const bodyDofs = traj.body_dofs || nodeOrder.map(() => 2);
    const pos = {};
    let qi = 0;
    for (let ni = 0; ni < nodeOrder.length; ni++) {
      const dof = bodyDofs[ni] || 2;
      pos[nodeOrder[ni]] = {
        x: q[qi],
        y: q[qi + 1],
        theta: dof >= 3 ? q[qi + 2] : 0,
      };
      qi += dof;
    }
    for (const [eid, ent] of this.sm.entities) {
      if (!pos[eid]) {
        pos[eid] = { x: ent.x, y: ent.y, theta: ent.theta || 0 };
      }
    }

    const ctx = this.ctx;
    for (const [id, edge] of this.sm.edges) {
      const pA = pos[edge.from];
      const pB = edge.to ? pos[edge.to] : null;
      if (!pA || (edge.to && !pB)) continue;
      const sA = this.camera.worldToScreen(pA.x, pA.y);
      const sB = this.camera.worldToScreen(pB.x, pB.y);
      switch (edge.type) {
        case 'IdealRod': Entities.drawIdealRod(ctx, sA.x, sA.y, sB.x, sB.y, false, false); break;
        case 'IdealSpring': Entities.drawIdealSpring(ctx, sA.x, sA.y, sB.x, sB.y, false, false); break;
        case 'SmoothRail': Entities.drawSmoothRail(ctx, sA.x, sA.y, sB.x, sB.y, false, false); break;
        case 'HingeJoint': Entities.drawHingeJoint(ctx, sA.x, sA.y, false, false); break;
        case 'DistanceSum': {
          const vE = edge.params?.via_id && this.sm.getEntity(edge.params.via_id);
          if (vE) Entities.drawDistanceSum(ctx, sA.x, sA.y, sB.x, sB.y, this.camera.worldToScreen(vE.x, vE.y), false, false);
          break;
        }
        case 'AngleConstraint':
          Entities.drawAngleConstraint(ctx, sA.x, sA.y, sB.x, sB.y, edge.params?.angle || 0, false, false); break;
      }
    }

    for (const nid of nodeOrder) {
      const p = pos[nid];
      if (!p) continue;
      const s = this.camera.worldToScreen(p.x, p.y);
      const ent = this.sm.getEntity(nid);
      if (ent && ent.type === 'RigidBody') {
        const len = (ent.params?.length || 2);
        const wid = (ent.params?.width || 0.5);
        const shape = ent.params?.shape || 'rect';
        Entities.drawRigidBody(ctx, s.x, s.y, p.theta, false, false, null, {
          length: len * this.camera.zoom,
          width: wid * this.camera.zoom,
          shape,
        });
      } else {
        Entities.drawMassPoint(ctx, s.x, s.y, false, false);
      }
    }

    ctx.fillStyle = '#656d76';
    ctx.font = '13px -apple-system, sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText(`t = ${this.sm.playhead.toFixed(2)}s`, 16, 28);
  }
}
