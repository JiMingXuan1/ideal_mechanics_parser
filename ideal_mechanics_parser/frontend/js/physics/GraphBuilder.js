export class GraphBuilder {
  constructor(stateMachine) {
    this.sm = stateMachine;
  }

  build() {
    const env = {
      view_plane: this.sm.viewPlane,
      gravity: this.sm.viewPlane === 'XZ' ? 9.81 : 0,
      time_step: 0.01,
      duration: this.sm._simDuration || 300,
    };

    const nodes = [];
    for (const [id, entity] of this.sm.entities) {
      if (entity.type === 'Anchor') {
        const params = {};
        if (entity.params?.x_expr) params.x_expr = entity.params.x_expr;
        if (entity.params?.y_expr) params.y_expr = entity.params.y_expr;
        const n = { id, type: 'Anchor', init_pos: [entity.x, entity.y] };
        if (Object.keys(params).length) n.params = params;
        nodes.push(n);
      } else if (entity.type === 'RigidBody') {
        nodes.push({
          id, type: 'RigidBody',
          params: { m: entity.params.m || 1.0, I: entity.params.I || 0.0 },
          init_state: {
            x: entity.x, y: entity.y, theta: entity.theta || 0,
            vx: entity.vx || 0, vy: entity.vy || 0, omega: entity.omega || 0,
          },
        });
      } else {
        const params = { m: entity.params.m || 1.0 };
        if (entity.params?.external_force_x_expr) params.external_force_x_expr = entity.params.external_force_x_expr;
        if (entity.params?.external_force_y_expr) params.external_force_y_expr = entity.params.external_force_y_expr;
        nodes.push({
          id, type: 'MassPoint',
          params,
          init_state: {
            x: entity.x, y: entity.y,
            vx: entity.vx || 0, vy: entity.vy || 0,
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
