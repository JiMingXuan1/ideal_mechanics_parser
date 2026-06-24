"""Frontend collision test: use fetch directly to bypass streaming issues."""
import os, sys, json, subprocess, time, socket
import pytest
from playwright.sync_api import sync_playwright

SERVER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
SERVER_SCRIPT = os.path.join(SERVER_DIR, "server.py")


def _free_port():
    s = socket.socket()
    s.bind(("", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture(scope="module")
def server():
    port = _free_port()
    proc = subprocess.Popen(
        [sys.executable, SERVER_SCRIPT, str(port)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=SERVER_DIR,
    )
    for _ in range(200):
        try:
            import urllib.request
            urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=1)
            break
        except Exception:
            time.sleep(0.1)
    else:
        proc.kill()
        pytest.fail("Server did not start")
    yield port
    proc.kill()
    proc.wait(timeout=5)


@pytest.fixture
def page(server):
    with sync_playwright() as pw:
        browser = pw.chromium.launch(headless=True)
        ctx = browser.new_context(viewport={"width": 1280, "height": 720})
        p = ctx.new_page()
        errors = []
        p.on("pageerror", lambda e: errors.append(str(e)))
        p.on("console", lambda msg: errors.append(f"[{msg.type}] {msg.text}") if msg.type == "error" else None)
        p.goto(f"http://127.0.0.1:{server}/", wait_until="networkidle")
        time.sleep(0.5)
        yield p, errors, server
        browser.close()


class TestCollision:
    def test_backend_batch(self, server):
        """POST /solve with collision returns correct data."""
        import urllib.request
        top = {"system_env": {"view_plane": "XY", "gravity": 0, "time_step": 0.01, "duration": 2.0},
            "nodes": [
                {"id": "n1", "type": "MassPoint", "params": {"m": 1.0, "radius": 0.1},
                 "init_state": {"x": 0, "y": 0, "vx": 1.0, "vy": 0}},
                {"id": "n2", "type": "MassPoint", "params": {"m": 1.0, "radius": 0.1},
                 "init_state": {"x": 2, "y": 0, "vx": -1.0, "vy": 0}}],
            "edges": []}
        req = urllib.request.Request(f"http://127.0.0.1:{server}/solve",
            data=json.dumps(top).encode(), headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=10) as resp:
            r = json.loads(resp.read())
        assert r["qd"][0][0] == 1.0
        assert r["qd"][-1][0] < 0

    def test_fetch_batch_from_browser(self, page):
        """Use fetch() in browser to hit /solve with collision."""
        p, errors, server = page
        result = p.evaluate(f"""
        async () => {{
            const resp = await fetch('http://127.0.0.1:{server}/solve', {{
                method: 'POST',
                headers: {{'Content-Type': 'application/json'}},
                body: JSON.stringify({{
                    "system_env": {{"view_plane": "XY", "gravity": 0, "time_step": 0.01, "duration": 2.0}},
                    "nodes": [
                        {{"id": "n1", "type": "MassPoint", "params": {{"m": 1.0, "radius": 0.1}},
                         "init_state": {{"x": 0, "y": 0, "vx": 1.0, "vy": 0}}}},
                        {{"id": "n2", "type": "MassPoint", "params": {{"m": 1.0, "radius": 0.1}},
                         "init_state": {{"x": 2, "y": 0, "vx": -1.0, "vy": 0}}}}
                    ],
                    "edges": []
                }})
            }});
            const data = await resp.json();
            return {{
                frames: data.t.length,
                first_vx: data.qd[0][0],
                last_vx: data.qd[data.qd.length-1][0],
                first_vx2: data.qd[0][2],
                last_vx2: data.qd[data.qd.length-1][2],
            }};
        }}
        """)
        assert result["frames"] > 5
        assert result["first_vx"] == 1.0
        assert result["last_vx"] < 0

    def test_fetch_stream_from_browser(self, page):
        """Use fetch() in browser to hit /solve/stream and read SSE."""
        p, errors, server = page
        result = p.evaluate(f"""
        async () => {{
            const resp = await fetch('http://127.0.0.1:{server}/solve/stream', {{
                method: 'POST',
                headers: {{'Content-Type': 'application/json'}},
                body: JSON.stringify({{
                    "system_env": {{"view_plane": "XY", "gravity": 0, "time_step": 0.01, "duration": 2.0}},
                    "nodes": [
                        {{"id": "n1", "type": "MassPoint", "params": {{"m": 1.0, "radius": 0.1}},
                         "init_state": {{"x": 0, "y": 0, "vx": 1.0, "vy": 0}}}},
                        {{"id": "n2", "type": "MassPoint", "params": {{"m": 1.0, "radius": 0.1}},
                         "init_state": {{"x": 2, "y": 0, "vx": -1.0, "vy": 0}}}}
                    ],
                    "edges": []
                }})
            }});
            const reader = resp.body.getReader();
            const decoder = new TextDecoder();
            let allData = [];
            while (true) {{
                const {{ done, value }} = await reader.read();
                if (done) break;
                const text = decoder.decode(value, {{stream: true}});
                for (const line of text.split('\\n')) {{
                    if (line.startsWith('data: ')) {{
                        allData.push(JSON.parse(line.slice(6)));
                    }}
                }}
            }}
            return {{
                messages: allData.length,
                hasTrajectory: allData.some(m => m.q),
                hasEvent: allData.some(m => m.event),
                firstVx: allData.find(m => m.qd)?.['qd']?.[0]?.[0],
            }};
        }}
        """)
        assert result["messages"] >= 3, f"Expected >=3 SSE messages, got {result['messages']}"
        assert result["hasTrajectory"], "No trajectory chunk"
        assert result["firstVx"] == 1.0, f"First vx should be 1.0, got {result['firstVx']}"
