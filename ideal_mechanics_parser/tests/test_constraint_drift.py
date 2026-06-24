import numpy as np
from core.engine import Engine


def test_rod_length_tolerance():
    topology = {
        "system_env": {
            "view_plane": "XZ",
            "gravity": 9.81,
            "time_step": 0.01,
            "duration": 10.0,
        },
        "nodes": [
            {"id": "n1", "type": "Anchor", "init_pos": [0.0, 10.0]},
            {
                "id": "n2",
                "type": "MassPoint",
                "params": {"m": 1.0},
                "init_state": {"x": 3.0, "y": 7.0, "vx": 0.0, "vy": 0.0},
            },
            {
                "id": "n3",
                "type": "MassPoint",
                "params": {"m": 1.0},
                "init_state": {"x": 6.0, "y": 5.0, "vx": 0.0, "vy": 0.0},
            },
            {
                "id": "n4",
                "type": "MassPoint",
                "params": {"m": 1.0},
                "init_state": {"x": 8.0, "y": 2.0, "vx": 0.0, "vy": 0.0},
            },
        ],
        "edges": [
            {"id": "e1", "type": "IdealRod", "from": "n1", "to": "n2", "params": {"length": 5.0}},
            {"id": "e2", "type": "IdealRod", "from": "n2", "to": "n3", "params": {"length": 5.0}},
            {"id": "e3", "type": "IdealRod", "from": "n3", "to": "n4", "params": {"length": 5.0}},
        ],
    }

    engine = Engine(topology)
    result = engine.run()

    q = np.array(result["q"])
    point_ids = {p.id: p.idx for p in engine.points}

    rod_edges = [e for e in engine.edges if e.type == "IdealRod"]
    for e in rod_edges:
        if e.from_id not in point_ids or e.to_id not in point_ids:
            continue
        fi = 2 * point_ids[e.from_id]
        ti = 2 * point_ids[e.to_id]

        dx = q[:, fi] - q[:, ti]
        dy = q[:, fi + 1] - q[:, ti + 1]
        d = np.sqrt(dx**2 + dy**2)
        errors = np.abs(d - e.length)

        max_err = errors.max()
        assert max_err < 1e-5, (
            f"Rod {e.id}: max length error {max_err:.2e} exceeds 1e-5"
        )
