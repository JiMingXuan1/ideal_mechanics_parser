"""Expression injection: moving anchors and external force expressions."""

import numpy as np
from core.engine import Engine


def test_moving_anchor_expression():
    """Anchor with x_expr moves horizontally, pendulum follows.

    Moving anchor at x(t)=t^2, y=5, pendulum rod length=5.
    Mass point should track the anchor's motion.
    """
    topology = {
        "system_env": {"view_plane": "XZ", "gravity": 9.81, "duration": 2.0, "time_step": 0.01},
        "nodes": [
            {"id": "n1", "type": "Anchor", "init_pos": [0.0, 5.0],
             "params": {"x_expr": "1.0 * t**2", "y_expr": "5.0"}},
            {"id": "n2", "type": "MassPoint", "params": {"m": 1.0},
             "init_state": {"x": 3.0, "y": 1.0, "vx": 0.0, "vy": 0.0}},
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


def test_external_force_x_drives_mass():
    """MassPoint with external_force_x_expr receives a driving force.

    F = sin(2*pi*t) on a free mass in XY plane.
    A free mass under sinusoidal force drifts (no restoring force),
    but the velocity should oscillate with the driving frequency.
    """
    topology = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 3.0, "time_step": 0.01},
        "nodes": [{
            "id": "m1", "type": "MassPoint", "params": {"m": 1.0,
                "external_force_x_expr": "sin(2 * pi * t)"},
            "init_state": {"x": 0.0, "y": 0.0, "vx": 0.0, "vy": 0.0},
        }],
        "edges": [],
    }

    engine = Engine(topology)
    result = engine.run()

    qd = np.array(result["qd"])
    vx = qd[:, 0]

    assert np.all(np.isfinite(qd)), "NaN with external force"
    assert np.any(np.abs(vx) > 0.01), "Mass velocity should change"

    # Under F=sin(2*pi*t), m=1: v(t) = -(1/(2*pi))*cos(2*pi*t) + 1/(2*pi)
    # v oscillates, crosses zero at t where cos(2*pi*t) = 1, i.e., t=0,1,2...
    zero_crossings = np.where(np.diff(np.sign(vx)))[0]
    assert len(zero_crossings) >= 1, "Velocity should oscillate"


def test_external_force_y_gravity_cancel():
    """External force in Y cancels gravity: mass should stay stationary in XZ.

    F_y = +m*g (upward) counteracts gravity -m*g (downward) → net zero.
    Initial velocity zero → mass stays in place.
    """
    m = 2.0
    g = 9.81
    topology = {
        "system_env": {"view_plane": "XZ", "gravity": g, "duration": 1.0, "time_step": 0.01},
        "nodes": [{
            "id": "m1", "type": "MassPoint", "params": {"m": m,
                "external_force_y_expr": f"{m} * {g}"},
            "init_state": {"x": 0.0, "y": 5.0, "vx": 0.0, "vy": 0.0},
        }],
        "edges": [],
    }

    engine = Engine(topology)
    result = engine.run()

    q = np.array(result["q"])
    y = q[:, 1]

    assert abs(y[-1] - 5.0) < 0.01, f"Y drifted: {y[-1]:.4f} (expected 5.0)"


def test_external_force_streaming():
    """External forces work with streaming mode."""
    topology = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 1.0, "time_step": 0.05},
        "nodes": [{
            "id": "m1", "type": "MassPoint", "params": {"m": 1.0,
                "external_force_x_expr": "5.0"},
            "init_state": {"x": 0, "y": 0, "vx": 0, "vy": 0},
        }],
        "edges": [],
    }

    chunks = []
    Engine(topology).run_stream(on_chunk=lambda c: chunks.append(c), seg_duration=0.3)

    non_final = [c for c in chunks if not c.get("complete") and "error" not in c]
    assert len(non_final) > 0, "No streaming chunks"
    # With F=5 on m=1, a=5, x should be positive after 0.5s
    any_q = [c["q"] for c in non_final if "q" in c]
    if any_q:
        q = np.array(any_q)
        assert np.any(q[:, :, 0] > 0), "Mass should accelerate under force"


def test_moving_anchor_with_spring():
    """Moving anchor connected via spring: spring stretches as anchor moves."""
    topology = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 2.0, "time_step": 0.01},
        "nodes": [
            {"id": "a1", "type": "Anchor", "init_pos": [0, 0],
             "params": {"x_expr": "2.0 * t", "y_expr": "0"}},
            {"id": "m1", "type": "MassPoint", "params": {"m": 1.0},
             "init_state": {"x": 1.0, "y": 0, "vx": 0, "vy": 0}},
        ],
        "edges": [
            {"id": "s1", "type": "IdealSpring", "from": "a1", "to": "m1",
             "params": {"k": 10.0, "l0": 1.0}},
        ],
    }

    engine = Engine(topology)
    result = engine.run()

    q = np.array(result["q"])
    assert np.all(np.isfinite(q)), "NaN with moving anchor + spring"


def test_external_force_energy_change():
    """External force does work: kinetic energy should increase."""
    topology = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 1.0, "time_step": 0.01},
        "nodes": [{
            "id": "m1", "type": "MassPoint", "params": {"m": 1.0,
                "external_force_x_expr": "2.0"},
            "init_state": {"x": 0, "y": 0, "vx": 0, "vy": 0},
        }],
        "edges": [],
    }

    engine = Engine(topology)
    result = engine.run()

    qd = np.array(result["qd"])
    ke = 0.5 * 1.0 * (qd[:, 0]**2 + qd[:, 1]**2)

    # Constant force 2N on 1kg for 1s → v = 2 m/s, KE = 2J
    assert abs(ke[-1] - 2.0) < 0.05, f"Final KE {ke[-1]:.4f} ≠ 2.0"
    assert ke[-1] > ke[0], "KE should increase with applied force"
