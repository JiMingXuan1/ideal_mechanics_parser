import numpy as np
from core.engine import Engine


def test_moving_anchor_expression():
    topology = {
        "system_env": {
            "view_plane": "XZ",
            "gravity": 9.81,
            "time_step": 0.01,
            "duration": 2.0,
        },
        "nodes": [
            {
                "id": "n1",
                "type": "Anchor",
                "init_pos": [0.0, 5.0],
                "params": {"x_expr": "0.5 * 2.0 * t**2", "y_expr": "5.0"},
            },
            {
                "id": "n2",
                "type": "MassPoint",
                "params": {"m": 1.0},
                "init_state": {"x": 3.0, "y": 1.0, "vx": 0.0, "vy": 0.0},
            },
        ],
        "edges": [
            {"id": "e1", "type": "IdealRod", "from": "n1", "to": "n2", "params": {"length": 5.0}},
        ],
    }

    engine = Engine(topology)
    result = engine.run()

    q = np.array(result["q"])
    t = np.array(result["t"])

    assert np.all(np.isfinite(q)), "NaN detected with expression injection"
    assert len(t) > 1, "No time steps produced"
