import numpy as np
import pytest
from core.engine import Engine
from core.exceptions import ProjectionError


def test_projection_corrects_bad_initial_state():
    L = 3.0

    topology = {
        "system_env": {
            "view_plane": "XZ",
                "gravity": 9.81,
                "time_step": 0.01,
                "duration": 1.0,
            },
            "nodes": [
                {"id": "n1", "type": "Anchor", "init_pos": [0.0, 0.0]},
                {
                    "id": "n2",
                    "type": "MassPoint",
                    "params": {"m": 1.0},
                    "init_state": {"x": 5.0, "y": 0.0, "vx": 0.0, "vy": 0.0},
            },
        ],
        "edges": [
            {"id": "e1", "type": "IdealRod", "from": "n1", "to": "n2", "params": {"length": L}},
        ],
    }

    engine = Engine(topology)
    result = engine.run()

    q = np.array(result["q"])
    x, y = q[0, 0], q[0, 1]
    d = np.sqrt(x**2 + y**2)
    assert abs(d - L) < 1e-10, (
        f"Projection did not correct initial state: distance={d}, expected={L}"
    )


def test_projection_raises_on_unsalvageable():
    topology = {
        "system_env": {
            "view_plane": "XZ",
                "gravity": 9.81,
                "time_step": 0.01,
                "duration": 1.0,
            },
            "nodes": [
                {"id": "n1", "type": "Anchor", "init_pos": [0.0, 0.0]},
                {
                    "id": "n2",
                    "type": "MassPoint",
                    "params": {"m": 1.0},
                    "init_state": {"x": 2.0, "y": 2.0, "vx": 0.0, "vy": 0.0},
            },
            {"id": "n3", "type": "Anchor", "init_pos": [0.0, 0.0]},
        ],
        "edges": [
            {"id": "e1", "type": "IdealRod", "from": "n1", "to": "n2", "params": {"length": 3.0}},
            {"id": "e2", "type": "IdealRod", "from": "n3", "to": "n2", "params": {"length": 5.0}},
        ],
    }

    engine = Engine(topology)
    with pytest.raises(ProjectionError):
        engine.run()
