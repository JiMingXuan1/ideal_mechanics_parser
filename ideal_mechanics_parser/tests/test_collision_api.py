"""Server-level collision test: verify POST /solve returns correct data for events.

Tests that the server properly routes eventful topologies to run_events()
and returns combined trajectory data.
"""

import json
import threading
import time
import urllib.request
import socket
import pytest
import numpy as np


@pytest.fixture(scope="module")
def server():
    from server import Handler
    import http.server
    s = socket.socket()
    s.bind(('', 0))
    port = s.getsockname()[1]
    s.close()
    sv = http.server.HTTPServer(('', port), Handler)
    t = threading.Thread(target=sv.serve_forever, daemon=True)
    t.start()
    time.sleep(0.5)
    yield port
    sv.shutdown()


def _post(port, body):
    data = json.dumps(body).encode()
    req = urllib.request.Request(
        f"http://localhost:{port}/solve",
        data=data,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            return json.loads(resp.read().decode())
    except urllib.error.HTTPError as e:
        return {"status": e.code, "body": e.read().decode(errors="replace")}


class TestCollisionBatch:

    def test_collision_batch_returns_trajectory(self, server):
        """POST /solve with collision topology returns trajectory data."""
        top = {
            "system_env": {"view_plane": "XY", "gravity": 0, "duration": 2.0, "time_step": 0.01,
                           "max_mutations": 10},
            "nodes": [
                {"id": "m1", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
                 "init_state": {"x": 0, "y": 0, "vx": 1, "vy": 0}},
                {"id": "m2", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
                 "init_state": {"x": 2, "y": 0, "vx": -1, "vy": 0}},
            ],
            "edges": [],
        }
        result = _post(server, top)
        assert "t" in result
        assert "q" in result
        assert len(result["t"]) > 0
        assert result["node_order"] == ["m1", "m2"]

    def test_collision_bounce_detected(self, server):
        """Elastic collision: velocities should change direction after impact."""
        top = {
            "system_env": {"view_plane": "XY", "gravity": 0, "duration": 2.0, "time_step": 0.01,
                           "max_mutations": 10},
            "nodes": [
                {"id": "m1", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
                 "init_state": {"x": 0, "y": 0, "vx": 1, "vy": 0}},
                {"id": "m2", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
                 "init_state": {"x": 2, "y": 0, "vx": -1, "vy": 0}},
            ],
            "edges": [],
        }
        result = _post(server, top)
        qd = np.array(result["qd"])
        vx1, vx2 = qd[:, 0], qd[:, 2]
        # Before collision: vx1=+1, vx2=-1
        # After elastic collision with equal masses: velocities swap
        # vx1 should end negative, vx2 should end positive
        assert vx1[-1] < 0, f"m1 should bounce back, ended at {vx1[-1]:.3f}"
        assert vx2[-1] > 0, f"m2 should bounce back, ended at {vx2[-1]:.3f}"

    def test_no_events_falls_to_streaming(self, server):
        """Topology without radius still uses streaming (no events)."""
        top = {
            "system_env": {"view_plane": "XY", "gravity": 0, "duration": 1.0, "time_step": 0.05},
            "nodes": [
                {"id": "m1", "type": "MassPoint", "params": {"m": 1},
                 "init_state": {"x": 0, "y": 0, "vx": 1, "vy": 0}},
            ],
            "edges": [],
        }
        result = _post(server, top)
        assert "t" in result
        assert result["body_dofs"] == [2]
