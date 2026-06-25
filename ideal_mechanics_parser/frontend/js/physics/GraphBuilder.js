export class GraphBuilder {
  constructor(stateMachine) {
    this.sm = stateMachine;
  }

  _n(v, fallback = 0) {
    const n = Number(v);
    return Number.isFinite(n) ? n : fallback;
  }

  build() {
    const env = {
      view_plane: this.sm.viewPlane,
      gravity: this.sm.viewPlane === 'XZ' ? 9.81 : 0,
      time_step: 0.01,
      duration: this.sm._simDuration || 300,
    };
    if (this.sm.gravitationEnabled) {
      env.gravitation = { enabled: true, G: 1.0, epsilon: 0.001 };
    }

    const nodes = [];
    for (const [id, entity] of this.sm.entities) {
      if (entity.type === 'Anchor') {
        const params = {};
        if (entity.params?.x_expr) params.x_expr = entity.params.x_expr;
        if (entity.params?.y_expr) params.y_expr = entity.params.y_expr;
        const n = { id, type: 'Anchor', init_pos: [this._n(entity.x), this._n(entity.y)] };
        if (Object.keys(params).length) n.params = params;
        nodes.push(n);
      } else if (entity.type === 'RigidBody') {
        nodes.push({
          id, type: 'RigidBody',
          params: {
            m: this._n(entity.params.m, 1),
            shape: entity.params.shape || 'rect',
            length: this._n(entity.params.length, 2),
            width: this._n(entity.params.width, 0.5),
          },
          init_state: {
            x: this._n(entity.x), y: this._n(entity.y),
            theta: this._n(entity.theta),
            vx: this._n(entity.vx), vy: this._n(entity.vy),
            omega: this._n(entity.omega),
          },
        });
      } else {
        const params = { m: this._n(entity.params.m, 1), radius: this._n(entity.params.radius, 0.1) };
        if (entity.params?.external_force_x_expr) params.external_force_x_expr = entity.params.external_force_x_expr;
        if (entity.params?.external_force_y_expr) params.external_force_y_expr = entity.params.external_force_y_expr;
        nodes.push({
          id, type: 'MassPoint',
          params,
          init_state: {
            x: this._n(entity.x), y: this._n(entity.y),
            vx: this._n(entity.vx), vy: this._n(entity.vy),
          },
        });
      }
    }

    const edges = [];
    for (const [id, edge] of this.sm.edges) {
      edges.push({
        id,
        type: edge.type,
        from: edge.from,
        to: edge.to,
        params: { ...edge.params },
      });
    }

    return { system_env: env, nodes, edges };
  }
}
