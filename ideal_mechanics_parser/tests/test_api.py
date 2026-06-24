"""HTTP API tests: batch /solve and streaming /solve/stream endpoints.

Starts a real server subprocess and sends HTTP requests with urllib.
"""

import json
import time
import subprocess
import urllib.request
import urllib.error
import threading
import pytest
import numpy as np

SERVER_PORT = 18923  # unlikely to conflict
SERVER_URL = f"http://127.0.0.1:{SERVER_PORT}"
SERVER_SCRIPT = "server.py"


# ─── Fixtures ────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def server():
    """Start the HTTP server in a subprocess, yield when ready, kill on teardown."""
    proc = subprocess.Popen(
            ["python", SERVER_SCRIPT, str(SERVER_PORT)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            cwd="C:/Users/Ji20081202/Documents/IMP/ideal_mechanics_parser",
        )

    # Wait for server to be ready (up to 10 s)
    for _ in range(100):
        try:
            with urllib.request.urlopen(f"{SERVER_URL}/", timeout=1):
                break
        except (urllib.error.URLError, ConnectionResetError, OSError):
            time.sleep(0.1)
    else:
        proc.kill()
        out, err = proc.communicate()
        pytest.fail(f"Server failed to start:\n{err.decode(errors='replace')}")

    yield

    proc.kill()
    proc.wait(timeout=5)


def _post(path, body):
    """POST JSON to path, return parsed JSON response."""
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        f"{SERVER_URL}{path}",
        data=data,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return {"status": e.code, "body": e.read().decode("utf-8", errors="replace")}


def _post_stream(path, body):
    """POST JSON to SSE endpoint, return list of parsed SSE messages."""
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        f"{SERVER_URL}{path}",
        data=data,
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=60) as resp:
        raw = resp.read().decode("utf-8")
    messages = []
    for line in raw.strip().split("\n"):
        if line.startswith("data: "):
            messages.append(json.loads(line[6:]))
    return messages


# ─── Topology helpers ────────────────────────────────────────────────

def free_body(body_id="b1", m=1.0, I=0.167, duration=1.0):
    return {
        "system_env": {"view_plane": "XY", "gravity": 0, "duration": duration, "time_step": 0.05},
        "nodes": [{
            "id": body_id, "type": "RigidBody",
            "params": {"m": m, "I": I},
            "init_state": {"x": 0, "y": 0, "theta": 0,
                           "vx": 0.5, "vy": 0.3, "omega": 1.0},
        }],
        "edges": [],
    }


def hinged_rod():
    theta = -1.47
    return {
        "system_env": {"view_plane": "XZ", "gravity": 9.81, "duration": 2.0, "time_step": 0.05},
        "nodes": [{
            "id": "rod", "type": "RigidBody",
            "params": {"m": 1.0, "I": 1.0 / 12.0},
            "init_state": {"x": 0.5 * np.cos(theta), "y": 0.5 * np.sin(theta),
                           "theta": theta, "vx": 0, "vy": 0, "omega": 0},
        }],
        "edges": [{
            "id": "h", "type": "HingeJoint", "from": "rod",
            "params": {"pivot": [-0.5, 0], "world": [0, 0]},
        }],
    }


# ─── Tests ───────────────────────────────────────────────────────────

class TestBatchSolve:

    def test_free_body_returns_valid_json(self, server):
        resp = _post("/solve", free_body())
        assert "t" in resp, f"Missing 't': {resp}"
        assert "q" in resp
        assert "qd" in resp
        assert "node_order" in resp
        assert resp["node_order"] == ["b1"]

    def test_free_body_has_body_dofs(self, server):
        resp = _post("/solve", free_body())
        assert resp["body_dofs"] == [3]

    def test_free_body_q_shape(self, server):
        resp = _post("/solve", free_body())
        q = resp["q"]
        n_t = len(resp["t"])
        assert len(q) == n_t, f"q length {len(q)} != t length {n_t}"
        assert len(q[0]) == 3, f"q[0] has {len(q[0])} DOFs, expected 3"

    def test_free_body_qd_shape(self, server):
        resp = _post("/solve", free_body())
        qd = resp["qd"]
        n_t = len(resp["t"])
        assert len(qd) == n_t
        assert len(qd[0]) == 3

    def test_hinged_rod_mixed_body_dofs(self, server):
        """Topology with both MassPoints and RigidBodies."""
        top = {
            "system_env": {"view_plane": "XY", "gravity": 0, "duration": 1.0, "time_step": 0.05},
            "nodes": [
                {"id": "mp", "type": "MassPoint", "params": {"m": 0.5},
                 "init_state": {"x": 0, "y": 0, "vx": 1, "vy": 0}},
                {"id": "rb", "type": "RigidBody", "params": {"m": 1, "I": 0.1},
                 "init_state": {"x": 1, "y": 0, "theta": 0, "vx": 0, "vy": 0, "omega": 0}},
            ],
            "edges": [],
        }
        resp = _post("/solve", top)
        assert resp["node_order"] == ["mp", "rb"]
        assert resp["body_dofs"] == [2, 3]

    def test_invalid_topology_returns_422(self, server):
        resp = _post("/solve", {"system_env": {}, "nodes": [], "edges": []})
        assert resp["status"] == 422, f"Expected 422, got {resp}"

    def test_unknown_node_type_returns_422(self, server):
        top = {
            "system_env": {"view_plane": "XY", "gravity": 0, "duration": 1.0, "time_step": 0.01},
            "nodes": [{"id": "x", "type": "UFO", "params": {}, "init_state": {}}],
            "edges": [],
        }
        resp = _post("/solve", top)
        assert resp["status"] == 422


class TestStreamingSolve:

    def test_stream_returns_chunks(self, server):
        msgs = _post_stream("/solve/stream", free_body(duration=0.5))
        assert len(msgs) > 0
        assert any(m.get("complete") for m in msgs)

    def test_stream_first_chunk_has_body_dofs(self, server):
        msgs = _post_stream("/solve/stream", free_body(duration=0.3, I=0.1))
        data_msgs = [m for m in msgs if "q" in m]
        assert len(data_msgs) > 0
        assert data_msgs[0].get("body_dofs") == [3]

    def test_stream_q_has_correct_shape(self, server):
        msgs = _post_stream("/solve/stream", free_body(duration=0.3))
        data_msgs = [m for m in msgs if "q" in m]
        for m in data_msgs:
            for row in m["q"]:
                assert len(row) == 3

    def test_stream_node_order(self, server):
        msgs = _post_stream("/solve/stream", free_body(duration=0.3))
        data_msgs = [m for m in msgs if "q" in m]
        for m in data_msgs:
            assert m["node_order"] == ["b1"]

    def test_stream_multiple_chunks(self, server):
        msgs = _post_stream("/solve/stream", free_body(duration=1.0))
        data_msgs = [m for m in msgs if "q" in m]
        assert len(data_msgs) >= 2, f"Expected ≥2 chunks, got {len(data_msgs)}"

    def test_stream_complete_signal(self, server):
        msgs = _post_stream("/solve/stream", free_body(duration=0.3))
        assert any(m.get("complete") for m in msgs), \
            f"No 'complete' signal in {len(msgs)} messages"

    def test_stream_error_on_invalid_topology(self, server):
        msgs = _post_stream("/solve/stream", {"x": 1})
        error_msgs = [m for m in msgs if "error" in m]
        assert len(error_msgs) > 0


class TestMixedTopology:

    def test_mass_point_and_rigid_body_batch(self, server):
        """MassPoint + RigidBody with HingeJoint connecting them."""
        top = {
            "system_env": {"view_plane": "XY", "gravity": 0, "duration": 1.0, "time_step": 0.05},
            "nodes": [
                {"id": "mp", "type": "MassPoint", "params": {"m": 0.5},
                 "init_state": {"x": 0, "y": 0, "vx": 0.5, "vy": 0}},
                {"id": "rb", "type": "RigidBody", "params": {"m": 1, "I": 0.1},
                 "init_state": {"x": 0, "y": 0, "theta": 0.3,
                                "vx": 0.5, "vy": 0, "omega": 0}},
            ],
            "edges": [{
                "id": "h", "type": "HingeJoint", "from": "rb", "to": "mp",
                "params": {"pivot": [0, 0]},
            }],
        }
        resp = _post("/solve", top)
        assert resp["node_order"] == ["mp", "rb"]
        assert resp["body_dofs"] == [2, 3]
        assert len(resp["t"]) > 0

    def test_two_rigid_bodies_batch(self, server):
        top = {
            "system_env": {"view_plane": "XY", "gravity": 0, "duration": 1.0, "time_step": 0.05},
            "nodes": [
                {"id": "b1", "type": "RigidBody", "params": {"m": 1, "I": 0.1},
                 "init_state": {"x": -0.5, "y": 0, "theta": 0,
                                "vx": 0.5, "vy": 0.3, "omega": 0.2}},
                {"id": "b2", "type": "RigidBody", "params": {"m": 2, "I": 0.2},
                 "init_state": {"x": 0.5, "y": 0, "theta": 0,
                                "vx": 0.5, "vy": 0.45, "omega": 0.1}},
            ],
            "edges": [{
                "id": "h", "type": "HingeJoint", "from": "b1", "to": "b2",
                "params": {"pivot": [0.5, 0], "pivot_b": [-0.5, 0]},
            }],
        }
        resp = _post("/solve", top)
        assert resp["body_dofs"] == [3, 3]
        assert len(resp["q"][0]) == 6  # 2 bodies × 3 DOFs

    def test_hinge_to_anchor_batch(self, server):
        theta = -1.47
        top = {
            "system_env": {"view_plane": "XZ", "gravity": 9.81, "duration": 1.0, "time_step": 0.05},
            "nodes": [
                {"id": "a1", "type": "Anchor", "init_pos": [0, 0]},
                {"id": "b1", "type": "RigidBody", "params": {"m": 1, "I": 1/12},
                 "init_state": {"x": 0.5*np.cos(theta), "y": 0.5*np.sin(theta),
                                "theta": theta, "vx": 0, "vy": 0, "omega": 0}},
            ],
            "edges": [{
                "id": "h", "type": "HingeJoint", "from": "b1", "to": "a1",
                "params": {"pivot": [-0.5, 0]},
            }],
        }
        resp = _post("/solve", top)
        assert resp["node_order"] == ["b1"]
        assert resp["body_dofs"] == [3]
        assert len(resp["t"]) > 0
