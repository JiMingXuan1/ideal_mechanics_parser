import numpy as np
import math
from core.engine import Engine


def _total_energy(q_traj, qd_traj, m1, m2, L1, L2, g):
    energies = []
    for i in range(len(q_traj)):
        x1, y1, x2, y2 = q_traj[i]
        vx1, vy1, vx2, vy2 = qd_traj[i]
        T = 0.5 * m1 * (vx1**2 + vy1**2) + 0.5 * m2 * (vx2**2 + vy2**2)
        V = m1 * g * y1 + m2 * g * y2
        energies.append(T + V)
    return np.array(energies)


def _total_momentum(qd_traj, m1, m2):
    momenta = []
    for i in range(len(qd_traj)):
        vx1, vy1, vx2, vy2 = qd_traj[i]
        px = m1 * vx1 + m2 * vx2
        py = m1 * vy1 + m2 * vy2
        momenta.append([px, py])
    return np.array(momenta)


def test_double_pendulum_conservation():
    L1 = 3.0
    L2 = 3.0
    m1 = 1.0
    m2 = 1.0
    g = 9.81

    topology = {
        "system_env": {
            "view_plane": "XZ",
            "gravity": g,
            "time_step": 0.01,
            "duration": 50.0,
        },
        "nodes": [
            {"id": "n1", "type": "Anchor", "init_pos": [0.0, 6.0]},
            {
                "id": "n2",
                "type": "MassPoint",
                "params": {"m": m1},
                "init_state": {"x": 2.0, "y": 3.0, "vx": 0.5, "vy": 0.0},
            },
            {
                "id": "n3",
                "type": "MassPoint",
                "params": {"m": m2},
                "init_state": {"x": 4.0, "y": 1.0, "vx": 0.0, "vy": 0.2},
            },
        ],
        "edges": [
            {"id": "e1", "type": "IdealRod", "from": "n1", "to": "n2", "params": {"length": L1}},
            {"id": "e2", "type": "IdealRod", "from": "n2", "to": "n3", "params": {"length": L2}},
        ],
    }

    engine = Engine(topology)
    result = engine.run()

    q = np.array(result["q"])
    qd = np.array(result["qd"])

    E = _total_energy(q, qd, m1, m2, L1, L2, g)
    E0 = E[0]
    E_range = E.max() - E.min()
    E_drift_ratio = E_range / abs(E0)
    assert E_drift_ratio < 0.05, (
        f"Energy conservation violated: drift={E_drift_ratio*100:.2f}%, "
        f"E0={E0:.6f}, Emax={E.max():.6f}, Emin={E.min():.6f}"
    )

    P = _total_momentum(qd, m1, m2)
    P0 = P[0]
    Px_drift = abs(P[:, 0] - P0[0]).max()
    Py_drift = abs(P[:, 1] - P0[1]).max()
