import numpy as np
from core.engine import Engine


def test_singular_jacobian_does_not_crash():
    P = (2.0, 2.0)
    A1 = (0.0, 0.0)
    A2 = (4.0, 0.0)
    A3 = (2.0, 4.0)
    import math
    L1 = math.sqrt((P[0]-A1[0])**2 + (P[1]-A1[1])**2)
    L2 = math.sqrt((P[0]-A2[0])**2 + (P[1]-A2[1])**2)
    L3 = math.sqrt((P[0]-A3[0])**2 + (P[1]-A3[1])**2)

    topology = {
        "system_env": {
            "view_plane": "XZ",
            "gravity": 9.81,
            "time_step": 0.01,
            "duration": 2.0,
        },
        "nodes": [
            {"id": "n1", "type": "Anchor", "init_pos": list(A1)},
            {"id": "n2", "type": "Anchor", "init_pos": list(A2)},
            {"id": "n3", "type": "Anchor", "init_pos": list(A3)},
            {
                "id": "n4",
                "type": "MassPoint",
                "params": {"m": 1.0},
                "init_state": {"x": P[0], "y": P[1], "vx": 0.0, "vy": 0.0},
            },
        ],
        "edges": [
            {"id": "e1", "type": "IdealRod", "from": "n1", "to": "n4", "params": {"length": L1}},
            {"id": "e2", "type": "IdealRod", "from": "n2", "to": "n4", "params": {"length": L2}},
            {"id": "e3", "type": "IdealRod", "from": "n3", "to": "n4", "params": {"length": L3}},
        ],
    }

    engine = Engine(topology)
    result = engine.run()

    q = np.array(result["q"])
    qd = np.array(result["qd"])

    assert np.all(np.isfinite(q)), "NaN detected in position trajectory"
    assert np.all(np.isfinite(qd)), "NaN detected in velocity trajectory"
