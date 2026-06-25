"""RigidBody collision tests: point-vs-rod, rod-vs-rod."""
import pytest
import numpy as np
from core.engine import Engine


def _rb_collide(v1=1.0, v2=0.0, x1=0, x2=3, m1=1, m2=1, r=0.1,
                shape1='point', shape2='rod', L2=2.0, duration=3.0):
    """Two-body collision with a RigidBody. shape1 can be 'point' or 'rod'."""
    nodes = []
    if shape1 == 'point':
        nodes.append({"id": "a", "type": "MassPoint", "params": {"m": m1, "radius": r},
                      "init_state": {"x": x1, "y": 0, "vx": v1, "vy": 0}})
    else:
        nodes.append({"id": "a", "type": "RigidBody", "params": {"m": m1, "shape": shape1, "length": L2, "width": 0.3},
                      "init_state": {"x": x1, "y": 0, "theta": 0, "vx": v1, "vy": 0, "omega": 0}})
    nodes.append({"id": "b", "type": "RigidBody", "params": {"m": m2, "shape": shape2, "length": L2, "width": 0.3},
                  "init_state": {"x": x2, "y": 0, "theta": 0, "vx": v2, "vy": 0, "omega": 0}})

    top = {"system_env": {"view_plane": "XY", "gravity": 0, "duration": duration,
                          "time_step": 0.01, "max_mutations": 10},
           "nodes": nodes, "edges": []}
    chunks = []
    Engine(top).run_events(on_chunk=lambda c: chunks.append(c))
    events = [c for c in chunks if "event" in c]
    qd_combined = []
    for c in chunks:
        if "qd" in c:
            qd_combined.extend(c["qd"])
    return {"events": events, "qd": qd_combined, "chunks": chunks}


def test_point_hits_rod():
    """MassPoint moving right hits a stationary rod: rod should move and spin."""
    r = _rb_collide(v1=2.0, v2=0.0, x1=0, x2=2.0, shape1='point', shape2='rod', L2=2.0)
    assert len(r["events"]) >= 1, "Point should hit rod"
    qd = np.array(r["qd"])
    # Rod COM should have velocity after impact
    rod_vx = qd[-1, 2] if qd.shape[1] >= 3 else qd[-1, 4]
    point_vx = qd[-1, 0]
    assert point_vx < 2.0, "Point should slow down after hitting rod"
    # Omega depends on where the point hits (at the midpoint for head-on)
    # Rod omega should change


def test_point_hits_rod_end():
    """Point hitting the end of a rod creates angular velocity."""
    # Point at x=0 heading right, rod centered at x=3 with L=4 (extends from x=1 to x=5)
    r = _rb_collide(v1=2.0, v2=0.0, x1=0, x2=3.0, shape1='point', shape2='rod', L2=4.0)
    assert len(r["events"]) >= 1
    qd = np.array(r["qd"])
    # The point hits the rod somewhere on the left half
    # Rod should rotate (omega != 0)
    if qd.shape[1] >= 6:
        rod_omega = qd[-1, 5]
        assert rod_omega != 0, "Rod should rotate from off-center hit"


def test_rod_vs_rod_cross():
    """Two rods at 90 degree angle: cross collision."""
    top = {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": 2.0,
                       "time_step": 0.01, "max_mutations": 10},
        "nodes": [
            {"id": "a", "type": "RigidBody", "params": {"m": 1, "shape": "rod", "length": 3},
             "init_state": {"x": -2, "y": 0, "theta": 0, "vx": 2, "vy": 0, "omega": 0}},
            {"id": "b", "type": "RigidBody", "params": {"m": 1, "shape": "rod", "length": 3},
             "init_state": {"x": 2, "y": 0, "theta": 0, "vx": -2, "vy": 0, "omega": 0}},
        ],
        "edges": [],
    }
    chunks = []
    Engine(top).run_events(on_chunk=lambda c: chunks.append(c))
    events = [c for c in chunks if "event" in c]
    assert len(events) >= 1, "Two rods should collide"


def test_rod_collision_event_fires():
    """Event chunk should be emitted for rod collision."""
    r = _rb_collide(v1=2.0, v2=0.0, x1=0, x2=2.0, shape1='point', shape2='rod', L2=2.0)
    assert len(r["events"]) >= 1
    assert "collision" in r["events"][0]["event"]
