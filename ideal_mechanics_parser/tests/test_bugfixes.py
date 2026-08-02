"""Regression tests for fixed bugs:

- SmoothRail constrained points[0] instead of the connected node
- SoftRope was completely inert when connected to an Anchor
- SoftRope could never slacken after tightening
- Resting contact exploded into 'Max mutations exceeded'
- Bodies starting inside a large anchor circle fell straight through
- README's Fy(t) = 'm*g' external force example threw SecurityError
"""

import pytest
import numpy as np
from core.engine import Engine


def _run_events(topology):
    chunks = []
    Engine(topology).run_events(on_chunk=lambda c: chunks.append(c))
    return chunks


def _trajectory(chunks):
    return np.concatenate([c["q"] for c in chunks if "t" in c])


# ─── SmoothRail targets the connected node ───────────────────────────

def test_smooth_rail_constrains_connected_node_not_first():
    top = {
        "system_env": {"view_plane": "XZ", "gravity": 9.81,
                       "time_step": 0.01, "duration": 1.0},
        "nodes": [
            {"id": "a1", "type": "Anchor", "init_pos": [0, 0]},
            {"id": "p1", "type": "MassPoint", "params": {"m": 1.0},
             "init_state": {"x": 2, "y": 0, "vx": 0, "vy": 0}},
            {"id": "p2", "type": "MassPoint", "params": {"m": 1.0},
             "init_state": {"x": 1, "y": 1, "vx": 0, "vy": 0}},
        ],
        "edges": [
            {"id": "rail", "type": "SmoothRail", "from": "a1", "to": "p2",
             "params": {"expr": "y - x**2"}},
        ],
    }
    r = Engine(top).run()
    q = np.array(r["q"])
    p1, p2 = q[:, 0:2], q[:, 2:4]
    assert abs(p2[-1, 1] - p2[-1, 0]**2) < 1e-6, \
        f"Connected p2 should be on the rail, got {p2[-1]}"
    assert abs(p1[-1, 1] - p1[-1, 0]**2) > 1e-3, \
        f"Unconnected p1 should be free, got {p1[-1]}"


# ─── SoftRope with an Anchor endpoint ────────────────────────────────

def test_soft_rope_anchor_tightens():
    top = {
        "system_env": {"view_plane": "XY", "gravity": 0,
                       "time_step": 0.01, "duration": 1.0, "max_mutations": 10},
        "nodes": [
            {"id": "a1", "type": "Anchor", "init_pos": [0, 0]},
            {"id": "p1", "type": "MassPoint", "params": {"m": 1.0},
             "init_state": {"x": 1, "y": 0, "vx": 3, "vy": 0}},
        ],
        "edges": [
            {"id": "rope", "type": "SoftRope", "from": "a1", "to": "p1",
             "params": {"length": 2.0}},
        ],
    }
    chunks = _run_events(top)
    events = [c for c in chunks if "event" in c]
    errors = [c for c in chunks if "error" in c]
    assert len(errors) == 0, f"Unexpected errors: {errors}"
    assert any("tighten" in c.get("event", "") for c in events), \
        "Anchor-to-point rope should tighten"
    q = _trajectory(chunks)
    dist = np.sqrt(q[:, 0]**2 + q[:, 1]**2)
    assert dist.max() <= 2.0 + 1e-6, f"Rope exceeded length: {dist.max():.4f}"


def test_soft_rope_pendulum_stays_taut():
    """A rope pendulum at rest at exactly length must stay taut and swing."""
    top = {
        "system_env": {"view_plane": "XZ", "gravity": 9.81,
                       "time_step": 0.01, "duration": 3.0, "max_mutations": 100},
        "nodes": [
            {"id": "a1", "type": "Anchor", "init_pos": [0, 0]},
            {"id": "p1", "type": "MassPoint", "params": {"m": 1.0},
             "init_state": {"x": 2.0, "y": 0, "vx": 0, "vy": 0}},
        ],
        "edges": [
            {"id": "rope", "type": "SoftRope", "from": "a1", "to": "p1",
             "params": {"length": 2.0}},
        ],
    }
    chunks = _run_events(top)
    errors = [c for c in chunks if "error" in c]
    assert len(errors) == 0, f"Unexpected errors: {errors}"
    q = _trajectory(chunks)
    dist = np.sqrt(q[:, 0]**2 + q[:, 1]**2)
    assert abs(dist.min() - 2.0) < 1e-6 and abs(dist.max() - 2.0) < 1e-6, \
        f"Pendulum rope should stay taut: dist in [{dist.min():.6f}, {dist.max():.6f}]"


def test_soft_rope_releases_when_pushed():
    """A tight rope must slacken when the constraint would push instead of
    pulling (mass above the anchor, gravity pulling it toward the anchor)."""
    top = {
        "system_env": {"view_plane": "XZ", "gravity": 9.81,
                       "time_step": 0.01, "duration": 2.0, "max_mutations": 100},
        "nodes": [
            {"id": "a1", "type": "Anchor", "init_pos": [0, 0]},
            {"id": "p1", "type": "MassPoint", "params": {"m": 1.0},
             "init_state": {"x": 0, "y": 2.0, "vx": 0, "vy": 0}},
        ],
        "edges": [
            {"id": "rope", "type": "SoftRope", "from": "a1", "to": "p1",
             "params": {"length": 2.0}},
        ],
    }
    chunks = _run_events(top)
    errors = [c for c in chunks if "error" in c]
    assert len(errors) == 0, f"Unexpected errors: {errors}"
    q = _trajectory(chunks)
    dist = np.sqrt(q[:, 0]**2 + q[:, 1]**2)
    assert dist.min() < 2.0 - 1e-3, \
        "Rope should go slack when pushed, but distance never dropped below length"


# ─── Resting contact ─────────────────────────────────────────────────

def test_resting_ball_on_floor_no_explosion():
    """Ball placed exactly on a floor anchor must rest, not explode into
    'Max mutations exceeded'."""
    top = {
        "system_env": {"view_plane": "XZ", "gravity": 9.81,
                       "time_step": 0.01, "duration": 2.0, "restitution": 1.0},
        "nodes": [
            {"id": "floor", "type": "Anchor", "init_pos": [0, 0],
             "params": {"radius": 10.0}},
            {"id": "ball", "type": "MassPoint", "params": {"m": 1.0, "radius": 0.5},
             "init_state": {"x": 0, "y": 10.5, "vx": 0, "vy": 0}},
        ],
        "edges": [],
    }
    chunks = _run_events(top)
    errors = [c for c in chunks if "error" in c]
    assert len(errors) == 0, f"Resting contact exploded: {errors}"
    q = _trajectory(chunks)
    assert abs(q[-1, 1] - 10.5) < 0.05, f"Ball should rest on the floor, y={q[-1,1]:.3f}"


def test_ball_inside_floor_circle_does_not_fall_through():
    """Ball dropped from inside a large anchor circle must bounce off the
    inner wall and stay inside, not fall through."""
    top = {
        "system_env": {"view_plane": "XZ", "gravity": 9.81,
                       "time_step": 0.01, "duration": 3.0, "restitution": 1.0,
                       "max_mutations": 100},
        "nodes": [
            {"id": "floor", "type": "Anchor", "init_pos": [0, 0],
             "params": {"radius": 10.0}},
            {"id": "ball", "type": "MassPoint", "params": {"m": 1.0, "radius": 0.5},
             "init_state": {"x": 0, "y": 5, "vx": 0, "vy": 0}},
        ],
        "edges": [],
    }
    chunks = _run_events(top)
    errors = [c for c in chunks if "error" in c]
    assert len(errors) == 0, f"Unexpected errors: {errors}"
    q = _trajectory(chunks)
    assert q[-1, 1] > -10.5, f"Ball fell through the floor: y={q[-1,1]:.3f}"


def test_elastic_bounce_from_above_still_works():
    """Regression: a ball dropped onto the top of a floor circle bounces."""
    top = {
        "system_env": {"view_plane": "XZ", "gravity": 9.81,
                       "time_step": 0.01, "duration": 5.0, "restitution": 1.0},
        "nodes": [
            {"id": "floor", "type": "Anchor", "init_pos": [0, 0],
             "params": {"radius": 10.0}},
            {"id": "ball", "type": "MassPoint", "params": {"m": 1.0, "radius": 0.5},
             "init_state": {"x": 0, "y": 20, "vx": 0, "vy": 0}},
        ],
        "edges": [],
    }
    chunks = _run_events(top)
    events = [c for c in chunks if "event" in c]
    assert len(events) >= 2, "Ball should bounce repeatedly"
    expected_first = (2 * 9.5 / 9.81) ** 0.5
    assert abs(events[0]["t_event"] - expected_first) < 0.05


def test_fully_inelastic_contact_converts_to_rest():
    """e=0 head-on collision: bodies stick together at rest (via contact
    constraint) instead of re-triggering collision events."""
    top = {
        "system_env": {"view_plane": "XY", "gravity": 0,
                       "time_step": 0.01, "duration": 2.0,
                       "max_mutations": 10, "restitution": 0.0},
        "nodes": [
            {"id": "a", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
             "init_state": {"x": 0, "y": 0, "vx": 1.0, "vy": 0}},
            {"id": "b", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
             "init_state": {"x": 2, "y": 0, "vx": -1.0, "vy": 0}},
        ],
        "edges": [],
    }
    chunks = _run_events(top)
    errors = [c for c in chunks if "error" in c]
    assert len(errors) == 0, f"Unexpected errors: {errors}"
    q = _trajectory(chunks)
    assert abs(q[-1, 1]) < 1e-9
    # Bodies stay in contact (no re-collisions after sticking)
    assert len([c for c in chunks if "event" in c]) <= 2


# ─── Expression injection ────────────────────────────────────────────

def test_external_force_mg_expression():
    """README example Fy(t) = 'm*g' must cancel gravity exactly."""
    top = {
        "system_env": {"view_plane": "XZ", "gravity": 9.81,
                       "time_step": 0.01, "duration": 1.0},
        "nodes": [
            {"id": "p1", "type": "MassPoint", "params": {"m": 2.0,
             "external_force_y_expr": "m*g"},
             "init_state": {"x": 0, "y": 3, "vx": 0, "vy": 0}},
        ],
        "edges": [],
    }
    r = Engine(top).run()
    q = np.array(r["q"])
    assert abs(q[-1, 1] - 3.0) < 1e-6, f"Ball should levitate, y={q[-1,1]:.3f}"
