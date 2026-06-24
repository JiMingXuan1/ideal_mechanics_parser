"""Tests for event-driven simulation: collision detection and soft rope."""

import pytest
import numpy as np
from core.engine import Engine


# ─── Collision Detection ────────────────────────────────────────────

def test_collision_elastic_restitution():
    """Two mass points with radius approaching head-on bounce elastically (e=1).

    m1=1 at x=0, vx=1; m2=1 at x=3, vx=-1; both radius=0.5.
    Initial gap = 3-0-2*0.5 = 2.0.  Closing speed = 2 m/s.
    After collision, velocities should swap if equal masses.
    """
    top = _collision_topology(1.0, 0, 3, -1, e=1.0)
    chunks = _run_events(top)

    events = [c for c in chunks if "event" in c]
    assert len(events) >= 1, "No collision event fired"

    final = _final_trajectory(chunks)
    if final is not None:
        qd = np.array(final["qd"])
        # After elastic collision with equal masses: velocities swap
        assert abs(qd[-1, 0] - (-1.0)) < 0.1, f"m1 final vx {qd[-1,0]:.3f} != -1"
        assert abs(qd[-1, 2] - 1.0) < 0.1, f"m2 final vx {qd[-1,2]:.3f} != 1"


def test_collision_inelastic():
    """Inelastic collision (e=0.5): v_rel_after = -e * v_rel_before."""
    top = _collision_topology(1.0, 0, 3, -1, e=0.5)
    top["system_env"]["restitution"] = 0.5
    chunks = _run_events(top)

    events = [c for c in chunks if "event" in c]
    assert len(events) >= 1, "No collision event fired"

    final = _final_trajectory(chunks)
    if final is not None:
        qd = np.array(final["qd"])
        v1, v2 = qd[-1, 0], qd[-1, 2]
        vrel_after = v1 - v2
        # e = -(v1_after - v2_after) / (v1_before - v2_before)
        #   = -(v1_after - v2_after) / (1.0 - (-1.0))
        #   = -(v1_after - v2_after) / 2.0
        e_actual = -vrel_after / 2.0
        assert abs(e_actual - 0.5) < 0.05, f"Effective restitution {e_actual:.3f} != 0.5"


def test_collision_without_radius_no_event():
    """Bodies without radius should not trigger collision events."""
    top = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 2.0, "time_step": 0.01,
                       "max_mutations": 10},
        "nodes": [
            {"id": "m1", "type": "MassPoint", "params": {"m": 1},
             "init_state": {"x": 0, "y": 0, "vx": 1, "vy": 0}},
            {"id": "m2", "type": "MassPoint", "params": {"m": 1},
             "init_state": {"x": 2, "y": 0, "vx": -1, "vy": 0}},
        ],
        "edges": [],
    }
    chunks = _run_events(top)
    events = [c for c in chunks if "event" in c]
    assert len(events) == 0, "No radius should not trigger collision"


def test_collision_unequal_masses():
    """m1=2, vx=1 collides with m2=1, vx=-2 (equal momenta, opposite directions)."""
    top = _collision_topology(2.0, 0, 3, -2, m1=2, m2=1, e=1.0)
    chunks = _run_events(top)

    events = [c for c in chunks if "event" in c]
    assert len(events) >= 1

    final = _final_trajectory(chunks)
    if final is not None:
        qd = np.array(final["qd"])
        p1 = 2.0 * qd[:, 0]
        p2 = 1.0 * qd[:, 2]
        p_tot = p1 + p2
        drift = np.abs(p_tot - p_tot[0]).max()
        assert drift < 0.1, f"Momentum drift after collision: {drift:.3f}"


def test_collision_wall():
    """Mass point with radius hits an immovable wall (Anchor with radius)."""
    top = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 2.0, "time_step": 0.01,
                       "max_mutations": 10},
        "nodes": [
                {"id": "wall", "type": "Anchor", "init_pos": [3.0, 0.0], "params": {"radius": 0.5}},
            {"id": "ball", "type": "MassPoint", "params": {"m": 1, "radius": 0.5},
             "init_state": {"x": 0, "y": 0, "vx": 2, "vy": 0}},
        ],
        "edges": [],
    }
    chunks = _run_events(top)
    events = [c for c in chunks if "event" in c]
    assert len(events) >= 1, "Ball should hit wall anchor"

    final = _final_trajectory(chunks)
    if final is not None:
        qd = np.array(final["qd"])
        # After elastic collision with infinite mass: vx should reverse
        assert qd[-1, 0] < 0, f"Ball should bounce back, vx={qd[-1,0]:.3f}"


# ─── Soft Rope ──────────────────────────────────────────────────────

def test_soft_rope_tightens():
    """Two masses connected by soft rope: when separated past rope length,
    the rope tightens and constrains further separation.
    """
    top = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 2.0, "time_step": 0.01,
                       "max_mutations": 10},
        "nodes": [
            {"id": "m1", "type": "MassPoint", "params": {"m": 1},
             "init_state": {"x": 0, "y": 0, "vx": -0.5, "vy": 0}},
            {"id": "m2", "type": "MassPoint", "params": {"m": 1},
             "init_state": {"x": 2, "y": 0, "vx": 0.5, "vy": 0}},
        ],
        "edges": [{
            "id": "rope", "type": "SoftRope", "from": "m1", "to": "m2",
            "params": {"length": 2.5},
        }],
    }
    chunks = _run_events(top)
    events = [c for c in chunks if "event" in c]
    tighten_events = [c for c in events if "tighten" in c.get("event", "")]
    assert len(tighten_events) >= 1, "Rope should tighten"

    final = _final_trajectory(chunks)
    if final is not None:
        q = np.array(final["q"])
        dt = np.sqrt((q[:, 0] - q[:, 2])**2 + (q[:, 1] - q[:, 3])**2)
        max_dist = dt.max()
        assert max_dist <= 2.5 + 1e-6, f"Max distance {max_dist:.4f} exceeds rope length"


def test_soft_rope_tighten_inelastic_impulse():
    """When rope tightens, relative velocity along the rope is cancelled
    (perfectly inelastic tightening), conserving momentum.
    """
    top = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 1.0, "time_step": 0.01,
                       "max_mutations": 10},
        "nodes": [
            {"id": "m1", "type": "MassPoint", "params": {"m": 2},
             "init_state": {"x": 0, "y": 0, "vx": -0.5, "vy": 0}},
            {"id": "m2", "type": "MassPoint", "params": {"m": 1},
             "init_state": {"x": 2, "y": 0, "vx": 1.0, "vy": 0}},
        ],
        "edges": [{
            "id": "rope", "type": "SoftRope", "from": "m1", "to": "m2",
            "params": {"length": 2.5},
        }],
    }
    chunks = _run_events(top)
    events = [c for c in chunks if "event" in c]
    tighten = [c for c in events if "tighten" in c.get("event", "")]
    assert len(tighten) >= 1, "Rope should tighten"


def test_soft_rope_free_when_slack():
    """Before tightening, rope exerts no constraint (masses move freely)."""
    top = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 2.0, "time_step": 0.01,
                       "max_mutations": 10},
        "nodes": [
            {"id": "m1", "type": "MassPoint", "params": {"m": 1},
             "init_state": {"x": 0, "y": 0, "vx": -0.5, "vy": 0}},
            {"id": "m2", "type": "MassPoint", "params": {"m": 1},
             "init_state": {"x": 2, "y": 0, "vx": 0.5, "vy": 0}},
        ],
        "edges": [{
            "id": "rope", "type": "SoftRope", "from": "m1", "to": "m2",
            "params": {"length": 5.0},
        }],
    }
    chunks = _run_events(top)
    events = [c for c in chunks if "event" in c]
    assert len(events) == 0, "Rope should not tighten when slack length not reached"


def test_max_mutations_guard():
    """Exceeding max_mutations returns error."""
    top = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 10.0, "time_step": 0.01,
                       "max_mutations": 1},
        "nodes": [
            {"id": "m1", "type": "MassPoint", "params": {"m": 1},
             "init_state": {"x": 0, "y": 0, "vx": -2.0, "vy": 0}},
            {"id": "m2", "type": "MassPoint", "params": {"m": 1},
             "init_state": {"x": 2, "y": 0, "vx": 2.0, "vy": 0}},
        ],
        "edges": [{
            "id": "rope", "type": "SoftRope", "from": "m1", "to": "m2",
            "params": {"length": 2.5},
        }],
    }
    chunks = _run_events(top)
    errors = [c for c in chunks if "error" in c]
    assert len(errors) > 0, "Should hit max_mutations guard"


# ─── Helpers ────────────────────────────────────────────────────────

def _collision_topology(v1, x1, x2, v2, e=1.0, m1=1, m2=1):
    return {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 2.0, "time_step": 0.01,
                       "max_mutations": 10},
        "nodes": [
            {"id": "m1", "type": "MassPoint", "params": {"m": m1, "radius": 0.5},
             "init_state": {"x": x1, "y": 0, "vx": v1, "vy": 0}},
            {"id": "m2", "type": "MassPoint", "params": {"m": m2, "radius": 0.5},
             "init_state": {"x": x2, "y": 0, "vx": v2, "vy": 0}},
        ],
        "edges": [],
    }


def _run_events(topology):
    chunks = []
    Engine(topology).run_events(on_chunk=lambda c: chunks.append(c))
    return chunks


def _final_trajectory(chunks):
    for c in reversed(chunks):
        if "q" in c:
            return c
    return None
