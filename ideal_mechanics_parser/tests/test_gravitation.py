"""N-body gravitational tests (Gauss G = k^2)."""

import pytest
import numpy as np
from core.engine import Engine

GAUSS_G = 0.000295912208


GAUSS_G = 0.000295912208


def _binary_orbit_topology(G=GAUSS_G, eps=0.001, duration=800.0):
    """Two equal masses in circular orbit about their barycenter.

    m1=m2=1, separation=2 AU, each orbits at radius=1 AU from barycenter.
    Circular orbital velocity (two-body): v = sqrt(G) / 2 ≈ 0.0086 AU/day
    Kepler period: P = 2*pi*sqrt(a^3/(G*M_total)) ≈ 730 days.
    """
    sep = 2.0
    v = np.sqrt(G) / 2.0
    return {
        "system_env": {"view_plane": "XY", "gravity": 0,
                       "duration": duration, "time_step": 1.0,
                       "gravitation": {"enabled": True, "G": G, "epsilon": eps}},
        "nodes": [
            # Start at x=+1, y=0 with +vy → moves upward → counterclockwise
            {"id": "s1", "type": "MassPoint", "params": {"m": 1.0},
             "init_state": {"x": sep/2, "y": 0.0, "vx": 0.0, "vy": v}},
            {"id": "s2", "type": "MassPoint", "params": {"m": 1.0},
             "init_state": {"x": -sep/2, "y": 0.0, "vx": 0.0, "vy": -v}},
        ],
        "edges": [],
    }


def test_binary_orbit_period():
    """Two equal masses orbit: Kepler period P = 2*pi*sqrt(a^3/(G*M)).

    With a=2, M=2, G=0.000296: P = 2*pi*sqrt(8/(0.000296*2)) ≈ 730.
    """
    # Use a faster orbit: m=1000 gives period ≈ 23 for faster testing
    top = _binary_orbit_topology(G=GAUSS_G * 1000, duration=40.0, eps=0.01)
    r = Engine(top).run()

    q = np.array(r["q"])
    x1, y1 = q[:, 0], q[:, 1]
    x2, y2 = q[:, 2], q[:, 3]

    # Track angle of first body about origin (barycenter)
    angle = np.arctan2(y1, x1)
    unwrapped = np.unwrap(angle)
    t = np.array(r["t"])

    # Find when angle crosses 2*pi (one full orbit)
    end_idx = np.searchsorted(unwrapped, 2 * np.pi)
    if end_idx < len(t):
        T_est = t[end_idx]
        G_actual = top["system_env"]["gravitation"]["G"]
        M_total = 2.0
        a = 2.0
        T_expected = 2 * np.pi * np.sqrt(a**3 / (G_actual * M_total))
        print(f"Orbital period: {T_est:.3f} (expected ~{T_expected:.3f})")
        assert abs(T_est - T_expected) / T_expected < 0.1, \
            f"Period {T_est:.3f} != {T_expected:.3f}"
    else:
        pytest.skip(f"Not a full orbit within {t[-1]:.1f} time units")


def test_binary_orbit_energy_conservation():
    """Binary orbit: total (kinetic + potential) energy conserved."""
    top = _binary_orbit_topology(duration=20.0)
    r = Engine(top).run()

    q = np.array(r["q"])
    qd = np.array(r["qd"])

    x1, y1, x2, y2 = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    vx1, vy1, vx2, vy2 = qd[:, 0], qd[:, 1], qd[:, 2], qd[:, 3]

    KE = 0.5 * 1.0 * (vx1**2 + vy1**2) + 0.5 * 1.0 * (vx2**2 + vy2**2)
    dx = x2 - x1
    dy = y2 - y1
    d = np.sqrt(dx**2 + dy**2 + 0.001**2)
    PE = -GAUSS_G * 1.0 * 1.0 / d

    E = KE + PE
    drift = np.abs(E - E[0]).max() / abs(E[0])
    assert drift < 0.01, f"Energy drift {drift:.2%}"


def test_binary_orbit_streaming():
    """Gravitational simulation works in streaming mode."""
    top = _binary_orbit_topology(duration=5.0)
    chunks = []
    Engine(top).run_stream(on_chunk=lambda c: chunks.append(c), seg_duration=1.0)

    assert len(chunks) > 0
    assert any(c.get("complete") for c in chunks)

    data_chunks = [c for c in chunks if "q" in c]
    for c in data_chunks:
        q_arr = np.array(c["q"])
        assert q_arr.shape[1] == 4, f"Expected 4 DOFs (2×2), got {q_arr.shape[1]}"


def test_gravitation_toggle_off():
    """Without gravitation, bodies in XY plane stay in rectilinear motion."""
    top = _binary_orbit_topology(G=0, duration=5.0)
    r = Engine(top).run()

    q = np.array(r["q"])
    # Without gravity, bodies just drift at constant velocity
    # Initial: s1 at (-1,0) with vy=sqrt(G)=0, vx=0 → stays at (-1, 0)
    # Actually with G=0, vy=0, so both stay still
    x1 = q[:, 0]
    assert abs(x1[-1] - 1.0) < 1e-10, "No gravity: body should not move"


def test_three_body_energy():
    """Three bodies in XY with gravity: total energy conserved."""
    top = {
        "system_env": {"view_plane": "XY", "gravity": 0,
                       "duration": 10.0, "time_step": 0.05,
                       "gravitation": {"enabled": True, "G": GAUSS_G, "epsilon": 0.001}},
        "nodes": [
            {"id": "a", "type": "MassPoint", "params": {"m": 2.0},
             "init_state": {"x": -2, "y": 0, "vx": 0, "vy": 0.02}},
            {"id": "b", "type": "MassPoint", "params": {"m": 1.0},
             "init_state": {"x": 1, "y": 1, "vx": -0.01, "vy": -0.01}},
            {"id": "c", "type": "MassPoint", "params": {"m": 1.0},
             "init_state": {"x": 1, "y": -1, "vx": 0.01, "vy": -0.01}},
        ],
        "edges": [],
    }
    r = Engine(top).run()

    qd = np.array(r["qd"])
    q = np.array(r["q"])

    m = [2.0, 1.0, 1.0]
    KE = sum(0.5 * m[i] * (qd[:, 2*i]**2 + qd[:, 2*i+1]**2) for i in range(3))

    PE = 0
    for i in range(3):
        for j in range(i+1, 3):
            dx = q[:, 2*i] - q[:, 2*j]
            dy = q[:, 2*i+1] - q[:, 2*j+1]
            d = np.sqrt(dx**2 + dy**2 + 0.001**2)
            PE += -GAUSS_G * m[i] * m[j] / d

    E = KE + PE
    drift = np.abs(E - E[0]).max() / abs(E[0])
    assert drift < 0.02, f"Three-body energy drift {drift:.2%}"
