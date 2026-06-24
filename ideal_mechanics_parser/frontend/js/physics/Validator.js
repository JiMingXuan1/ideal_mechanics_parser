export class Validator {
  constructor(stateMachine) {
    this.sm = stateMachine;
  }

  validate() {
    const errors = [];

    const entityIds = new Set(this.sm.entities.keys());
    const connectedNodeIds = new Set();

    for (const [id, edge] of this.sm.edges) {
      if (!entityIds.has(edge.from)) {
        errors.push(`Edge "${id}" references unknown from-node "${edge.from}"`);
      }
      if (edge.to && !entityIds.has(edge.to)) {
        errors.push(`Edge "${id}" references unknown to-node "${edge.to}"`);
      }
      if (edge.to && edge.from === edge.to) {
        errors.push(`Edge "${id}" connects a node to itself`);
      }
      connectedNodeIds.add(edge.from);
      if (edge.to) connectedNodeIds.add(edge.to);
    }

    for (const [id, entity] of this.sm.entities) {
      if (!connectedNodeIds.has(id)) {
        errors.push(`Node "${id}" (${entity.type}) is isolated — no edges connected`);
      }
    }

    const edgePairs = new Set();
    for (const [id, edge] of this.sm.edges) {
      const key =
        edge.from < edge.to
          ? `${edge.from}-${edge.to}-${edge.type}`
          : `${edge.to}-${edge.from}-${edge.type}`;
      if (edgePairs.has(key)) {
        errors.push(`Duplicate edge "${id}": same pair already connected with type ${edge.type}`);
      }
      edgePairs.add(key);
    }

    return errors;
  }
}
