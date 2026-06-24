import json

VALID_EDGE_TYPES = {
    "IdealRod", "IdealSpring", "SmoothRail",
    "FixedCoordinate", "LinearRelation", "DistanceSum", "AngleConstraint",
}

def parse_json(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        data = json.load(f)

    topology = {
        "system_env": data.get("system_env", {}),
        "nodes": data.get("nodes", []),
        "edges": data.get("edges", []),
    }

    _validate(topology)
    return topology


def _validate(topology):
    env = topology["system_env"]
    assert "view_plane" in env, "Missing system_env.view_plane"
    assert env["view_plane"] in ("XY", "XZ"), "view_plane must be XY or XZ"
    assert "gravity" in env, "Missing system_env.gravity"
    assert "duration" in env, "Missing system_env.duration"

    node_ids = set()
    for n in topology["nodes"]:
        assert "id" in n, "Node missing id"
        assert "type" in n, f"Node {n['id']} missing type"
        assert n["type"] in ("Anchor", "MassPoint"), f"Unknown node type: {n['type']}"
        if n["type"] == "Anchor":
            assert "init_pos" in n, f"Anchor {n['id']} missing init_pos"
        node_ids.add(n["id"])

    for e in topology["edges"]:
        assert "id" in e, "Edge missing id"
        assert "type" in e, f"Edge {e['id']} missing type"
        assert e["type"] in VALID_EDGE_TYPES, f"Unknown edge type: {e['type']}"
        if "from" in e and e["from"] is not None:
            assert e["from"] in node_ids, f"Edge {e['id']}: from node {e['from']} not found"
        if "to" in e and e["to"] is not None:
            assert e["to"] in node_ids, f"Edge {e['id']}: to node {e['to']} not found"
        if e["type"] == "DistanceSum":
            via = e.get("params", {}).get("via_id")
            assert via is not None, f"DistanceSum {e['id']}: missing via_id in params"
            assert via in node_ids, f"DistanceSum {e['id']}: via node {via} not found"
