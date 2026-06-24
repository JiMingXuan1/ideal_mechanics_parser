import numpy as np
import math
from core.engine import Engine


def test_spring_oscillator_frequency():
    m = 1.0
    k = 100.0
    l0 = 0.5
    A = 0.5

    topology = {
        "system_env": {
            "view_plane": "XZ",
            "gravity": 0.0,
            "time_step": 0.001,
            "duration": 5.0,
        },
        "nodes": [
            {"id": "n1", "type": "Anchor", "init_pos": [0.0, 0.0]},
            {
                "id": "n2",
                "type": "MassPoint",
                "params": {"m": m},
                "init_state": {"x": l0 + A, "y": 0.0, "vx": 0.0, "vy": 0.0},
            },
        ],
        "edges": [
            {"id": "e1", "type": "IdealSpring", "from": "n1", "to": "n2", "params": {"k": k, "l0": l0}},
        ],
    }

    engine = Engine(topology)
    result = engine.run()

    t = np.array(result["t"])
    q = np.array(result["q"])
    x = q[:, 0]

    zero_crossings = []
    for i in range(1, len(x)):
        if x[i - 1] >= l0 and x[i] < l0:
            zero_crossings.append(t[i])

    if len(zero_crossings) < 3:
        raise AssertionError(f"Not enough zero crossings ({len(zero_crossings)}) to compute period")

    periods = np.diff(zero_crossings)
    T_num = np.mean(periods)
    omega_num = 2 * math.pi / T_num
    omega_theory = math.sqrt(k / m)

    assert abs(omega_num - omega_theory) < 0.01, (
        f"Frequency mismatch: numerical={omega_num:.4f}, theory={omega_theory:.4f}"
    )
