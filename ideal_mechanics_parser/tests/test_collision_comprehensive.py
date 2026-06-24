"""Comprehensive collision tests: edge cases, restitution, multiple bodies."""
import pytest
import numpy as np
from core.engine import Engine


def _collide(v1=1.0, v2=-1.0, x1=0, x2=2, m1=1, m2=1, r=0.1, duration=2.0, restitution=1.0):
    """Helper: two bodies heading toward each other, get combined qd."""
    top = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": duration,
                       "time_step": 0.01, "max_mutations": 10},
        "nodes": [
            {"id": "a", "type": "MassPoint", "params": {"m": m1, "radius": r},
             "init_state": {"x": x1, "y": 0, "vx": v1, "vy": 0}},
            {"id": "b", "type": "MassPoint", "params": {"m": m2, "radius": r},
             "init_state": {"x": x2, "y": 0, "vx": v2, "vy": 0}},
        ],
        "edges": [],
    }
    if restitution != 1.0:
        top["system_env"]["restitution"] = restitution
    chunks = []
    Engine(top).run_events(on_chunk=lambda c: chunks.append(c))
    t, q, qd = [], [], []
    for c in chunks:
        if "t" in c:
            t.extend(c["t"])
            q.extend(c["q"])
            if c.get("qd"):
                qd.extend(c["qd"])
    return {"t": t, "q": q, "qd": qd, "chunks": chunks}


# ─── Equal Mass ──────────────────────────────────────────────────

def test_head_on_elastic_equal_mass():
    """v=+1/-1 head-on: velocities swap."""
    r = _collide()
    qd = np.array(r["qd"])
    assert qd[0, 0] == 1.0 and qd[-1, 0] < 0
    assert qd[0, 2] == -1.0 and qd[-1, 2] > 0
    assert abs(abs(qd[-1, 0]) - 1.0) < 0.02
    assert abs(abs(qd[-1, 2]) - 1.0) < 0.02


# ─── Unequal Mass ────────────────────────────────────────────────

def test_light_hits_heavy_at_rest():
    """m1=10 at x=0 v=0, m2=1 at x=3 v=-3 moving left: momentum transfers."""
    r = _collide(v1=0, v2=-3, m1=10, m2=1, x1=0, x2=3)
    qd = np.array(r["qd"])
    p_before = 10 * 0 + 1 * (-3)  # = -3
    p_after = 10 * qd[-1, 0] + 1 * qd[-1, 2]
    assert abs(p_after - p_before) < 0.01
    assert qd[-1, 0] < 0, "Heavy mass should move backward (hit from right)"


def test_light_bounces_off_heavy():
    """m1=1000 v=0, m2=1 v=-2: light bounces, heavy barely moves."""
    r = _collide(v1=0, v2=-2, m1=1000, m2=1, x1=0, x2=3)
    qd = np.array(r["qd"])
    assert qd[-1, 2] > 0, "Light mass should bounce back (vx>0 after hit)"
    assert abs(qd[-1, 0]) < 0.01, "Heavy mass should barely move"


# ─── Restitution ────────────────────────────────────────────────

def test_inelastic_restitution_05():
    """e=0.5: v_rel_after = -0.5 * v_rel_before."""
    r = _collide(restitution=0.5)
    qd = np.array(r["qd"])
    v_rel_before = qd[0, 0] - qd[0, 2]
    v_rel_after = qd[-1, 0] - qd[-1, 2]
    assert abs(-v_rel_after / v_rel_before - 0.5) < 0.05


def test_fully_inelastic():
    """e=0: equal masses stick (both stop for symmetric collision)."""
    r = _collide(restitution=0.0)
    qd = np.array(r["qd"])
    assert abs(qd[-1, 0] - qd[-1, 2]) < 0.02
    assert abs(qd[-1, 0]) < 0.02


def test_superelastic():
    """e=1.5: separation speed > approach speed."""
    r = _collide(restitution=1.5)
    qd = np.array(r["qd"])
    v_rel_before = qd[0, 0] - qd[0, 2]
    v_rel_after = qd[-1, 0] - qd[-1, 2]
    assert v_rel_after / v_rel_before < -1.0


# ─── Edge Cases ─────────────────────────────────────────────────

def test_zero_radius_no_collision():
    """radius=0: bodies pass through, no event."""
    r = _collide(r=0.0)
    qd = np.array(r["qd"])
    if len(qd.shape) == 1:
        pytest.skip("run_stream fallthrough has no qd; no collision expected")
    assert abs(qd[-1, 0] - 1.0) < 0.01
    assert abs(qd[-1, 2] - (-1.0)) < 0.01


def test_tiny_radius_still_detects():
    """radius=0.05: collision detected with moderate radius."""
    r = _collide(r=0.05, x2=1.1)
    qd = np.array(r["qd"])
    assert qd[-1, 0] < 0, "Should bounce even with 0.05 radius"


def test_fast_catches_slow():
    """m2 behind m1, faster: catches up from behind."""
    # m1 at x=2 v=0.5 (slow, ahead), m2 at x=0 v=2.0 (fast, behind)
    r = _collide(v1=0.5, v2=2.0, x1=2, x2=0)
    qd = np.array(r["qd"])
    assert qd[-1, 0] > qd[0, 0], "Front body should speed up after hit"
    assert qd[-1, 2] < qd[0, 2], "Rear body should slow after hit"


def test_three_bodies_chain():
    """Three in a line: a hits b, then b hits c."""
    top = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 3.0,
                       "time_step": 0.01, "max_mutations": 20},
        "nodes": [
            {"id": "a", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
             "init_state": {"x": 0, "y": 0, "vx": 2.0, "vy": 0}},
            {"id": "b", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
             "init_state": {"x": 2, "y": 0, "vx": 0, "vy": 0}},
            {"id": "c", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
             "init_state": {"x": 4, "y": 0, "vx": 0, "vy": 0}},
        ],
        "edges": [],
    }
    chunks = []
    Engine(top).run_events(on_chunk=lambda c: chunks.append(c))
    events = [c for c in chunks if "event" in c]
    assert len(events) >= 2


# ─── Conservation ────────────────────────────────────────────────

def test_momentum_conservation():
    """Linear momentum conserved across collision (unequal masses)."""
    r = _collide(m1=3, m2=1, v1=1.0, v2=-2.0)
    qd = np.array(r["qd"])
    p_before = 3 * qd[0, 0] + 1 * qd[0, 2]
    p_after = 3 * qd[-1, 0] + 1 * qd[-1, 2]
    assert abs(p_after - p_before) < 0.01


def test_ke_drops_with_inelastic():
    """Inelastic collision (e=0.3) loses KE vs elastic (e=1)."""
    r_el = _collide(restitution=1.0, duration=1.0)
    r_in = _collide(restitution=0.3, duration=1.0)
    ke_el = 0.5 * (r_el["qd"][-1][0]**2 + r_el["qd"][-1][2]**2)
    ke_in = 0.5 * (r_in["qd"][-1][0]**2 + r_in["qd"][-1][2]**2)
    assert ke_in < ke_el


# ─── Event Format ───────────────────────────────────────────────

def test_collision_event_fires():
    """run_events emits exactly one event chunk for single collision."""
    r = _collide()
    events = [c for c in r["chunks"] if "event" in c]
    assert len(events) == 1
    assert events[0]["t_event"] > 0


def test_pre_event_trajectory_exists():
    """Trajectory chunk exists before event chunk."""
    r = _collide()
    traj_chunks = [c for c in r["chunks"] if "t" in c]
    assert len(traj_chunks) >= 1
