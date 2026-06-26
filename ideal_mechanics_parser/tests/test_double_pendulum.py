"""Tests for double pendulum: batch, streaming, and frontend consistency."""
import os, sys, json, subprocess, time, socket
import pytest
import urllib.request
import numpy as np

SERVER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")


def _free_port():
    s = socket.socket(); s.bind(("", 0)); p = s.getsockname()[1]; s.close(); return p


def _double_pendulum(duration=5.0):
    return {
        "system_env": {"view_plane": "XZ", "gravity": 9.81, "time_step": 0.01, "duration": duration},
        "nodes": [
            {"id": "a", "type": "Anchor", "init_pos": [0, 5]},
            {"id": "m1", "type": "MassPoint", "params": {"m": 1},
             "init_state": {"x": 1, "y": 3, "vx": 0, "vy": 0}},
            {"id": "m2", "type": "MassPoint", "params": {"m": 1},
             "init_state": {"x": 2, "y": 1, "vx": 0, "vy": 0}},
        ],
        "edges": [
            {"id": "r1", "type": "IdealRod", "from": "a", "to": "m1", "params": {"length": 2.236}},
            {"id": "r2", "type": "IdealRod", "from": "m1", "to": "m2", "params": {"length": 2.236}},
        ],
    }


@pytest.fixture(scope="module")
def server():
    port = _free_port()
    proc = subprocess.Popen(
        [sys.executable, os.path.join(SERVER_DIR, "server.py"), str(port)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=SERVER_DIR,
    )
    for _ in range(200):
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=1)
            break
        except Exception:
            time.sleep(0.1)
    else:
        proc.kill(); pytest.fail("Server not started")
    yield port
    proc.kill()


class TestDoublePendulum:

    def test_backend_engine(self):
        """Direct backend computation."""
        from core.engine import Engine
        r = Engine(_double_pendulum()).run()
        assert len(r["t"]) > 10

    def test_batch_endpoint(self, server):
        """POST /solve returns valid data."""
        top = _double_pendulum()
        data = json.dumps(top).encode()
        req = urllib.request.Request(f"http://127.0.0.1:{server}/solve",
            data=data, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=30) as resp:
            r = json.loads(resp.read())
        assert "t" in r
        assert "q" in r
        assert "qd" in r
        assert len(r["t"]) > 10

    def test_rod_constraint_holds(self, server):
        """Both rod constraints should hold throughout simulation."""
        from core.engine import Engine
        r = Engine(_double_pendulum()).run()
        q = np.array(r["q"])
        t = np.array(r["t"])

        # Rod 1: distance between anchor (0,5) and m1 should = 2.236
        d1 = np.sqrt((q[:, 0] - 0)**2 + (q[:, 1] - 5)**2)
        err1 = np.abs(d1 - 2.236).max()
        assert err1 < 1e-9, f"Rod1 error: {err1:.2e}"

        # Rod 2: distance between m1 and m2 should = 2.236
        d2 = np.sqrt((q[:, 0] - q[:, 2])**2 + (q[:, 1] - q[:, 3])**2)
        err2 = np.abs(d2 - 2.236).max()
        assert err2 < 1e-9, f"Rod2 error: {err2:.2e}"

    def test_energy_conservation(self, server):
        """Total mechanical energy drifts less than 2% over 5 seconds."""
        from core.engine import Engine
        r = Engine(_double_pendulum(duration=5.0)).run()
        q, qd = np.array(r["q"]), np.array(r["qd"])
        g, m = 9.81, 1.0
        ke = 0.5 * m * (qd[:, 0]**2 + qd[:, 1]**2 + qd[:, 2]**2 + qd[:, 3]**2)
        pe = m * g * (q[:, 1] + q[:, 3])
        E = ke + pe
        drift = np.abs(E - E[0]).max() / abs(E[0])
        assert drift < 0.02, f"Energy drift: {drift:.3%}"

    def test_streaming_via_server(self, server):
        """POST /solve/stream returns data (with short timeout for non-event)."""
        top = _double_pendulum(duration=2.0)
        data = json.dumps(top).encode()
        req = urllib.request.Request(f"http://127.0.0.1:{server}/solve/stream",
            data=data, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=30) as resp:
            raw = resp.read().decode()
        msgs = [json.loads(l[6:]) for l in raw.split("\n") if l.startswith("data: ")]
        assert len(msgs) >= 1
        assert any("q" in m for m in msgs), "No trajectory data in stream"

    def test_node_order_correct(self, server):
        """node_order should list MassPoints (not Anchor)."""
        from core.engine import Engine
        r = Engine(_double_pendulum()).run()
        assert r["node_order"] == ["m1", "m2"]
        assert r["body_dofs"] == [2, 2]

    def test_batch_via_browser(self, page_with_server):
        """Use fetch() in browser to hit /solve."""
        p, _, server = page_with_server
        result = p.evaluate(f"""
        async () => {{
            var top = {json.dumps(_double_pendulum())};
            var r = await fetch('http://127.0.0.1:{server}/solve', {{
                method:'POST', headers:{{'Content-Type':'application/json'}}, body:JSON.stringify(top)
            }});
            var d = await r.json();
            return {{ok:true, frames:d.t.length}};
        }}
        """)
        assert result["ok"]
        assert result["frames"] > 10


@pytest.fixture
def page_with_server(server):
    from playwright.sync_api import sync_playwright
    with sync_playwright() as pw:
        b = pw.chromium.launch(headless=True)
        ctx = b.new_context(viewport={"width": 1280, "height": 720})
        p = ctx.new_page()
        p.goto(f"http://127.0.0.1:{server}/", wait_until="domcontentloaded")
        p.evaluate(f"window.apiClient.base = 'http://127.0.0.1:{server}'")
        time.sleep(0.5)
        yield p, [], server
        b.close()
