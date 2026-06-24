import numpy as np
import math
from core.engine import Engine


def test_single_pendulum_period():
    L = 1.0
    g = 9.81
    theta0 = 0.05

    topology = {
        "system_env": {
            "view_plane": "XZ",
            "gravity": g,
            "time_step": 0.01,
            "duration": 10.0,
        },
        "nodes": [
            {"id": "n1", "type": "Anchor", "init_pos": [0.0, 0.0]},
            {
                "id": "n2",
                "type": "MassPoint",
                "params": {"m": 1.0},
                "init_state": {
                    "x": L * math.sin(theta0),
                    "y": -L * math.cos(theta0),
                    "vx": 0.0,
                    "vy": 0.0,
                },
            },
        ],
        "edges": [
            {"id": "e1", "type": "IdealRod", "from": "n1", "to": "n2", "params": {"length": L}},
        ],
    }

    engine = Engine(topology)
    result = engine.run()

    t = np.array(result["t"])
    qd = np.array(result["qd"])
    vx = qd[:, 0]

    zero_crossings = []
    for i in range(1, len(vx)):
        if vx[i - 1] <= 0 and vx[i] > 0:
            zero_crossings.append(t[i])

    if len(zero_crossings) < 3:
        raise AssertionError(f"Not enough zero crossings ({len(zero_crossings)}) to compute period")

    periods = np.diff(zero_crossings)
    T_num = np.mean(periods)
    T_theory = 2 * math.pi * math.sqrt(L / g)

    assert abs(T_num - T_theory) < 5e-3, (
        f"Period mismatch: numerical={T_num:.6f}, theory={T_theory:.6f}, diff={abs(T_num - T_theory):.2e}"
    )
