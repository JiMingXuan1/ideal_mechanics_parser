"""E2E user flow: toolbar clicks + Run + trajectory + collision."""
import os, sys, json, subprocess, time, socket
import pytest
from playwright.sync_api import sync_playwright

SERVER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

def _click_tool(page, label):
    page.evaluate('document.querySelectorAll(".tool-btn").forEach(b=>{var l=b.querySelector(".label");if(l&&l.textContent.includes("'+label+'"))b.click()})')

def _click_run(page):
    page.evaluate("document.querySelector('.run-btn')?.click()")

def _get_traj(page, timeout=30, min_frames=50):
    for _ in range(timeout * 10):
        d = page.evaluate("(()=>{var t=window.sm.trajectory;if(!t||t.t.length<"+str(min_frames)+")return null;return{f:t.t.length,last_t:t.t[t.t.length-1]}})()")
        if d: return d
        time.sleep(0.1)
    return None

def _server():
    s = socket.socket(); s.bind(("", 0)); p = s.getsockname()[1]; s.close()
    proc = subprocess.Popen([sys.executable, os.path.join(SERVER_DIR, "server.py"), str(p)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=SERVER_DIR)
    for _ in range(200):
        try:
            import urllib.request
            urllib.request.urlopen(f"http://127.0.0.1:{p}/", timeout=1)
            break
        except: time.sleep(0.1)
    else:
        proc.kill(); pytest.fail("No server")
    return proc, p


class TestUserFlow:

    def _flow(self, action):
        proc, port = _server()
        try:
            with sync_playwright() as pw:
                b = pw.chromium.launch(headless=True)
                p = b.new_page()
                p.goto(f"http://127.0.0.1:{port}/", wait_until="domcontentloaded")
                p.evaluate(f"window.apiClient.base='http://127.0.0.1:{port}'")
                time.sleep(0.5)
                action(p, port)
                b.close()
        finally:
            proc.kill()

    # --- Tests ---

    def test_toolbar_places_point(self):
        def fn(p, port):
            _click_tool(p, "Point")
            time.sleep(0.1)
            assert p.evaluate("window.sm.toolMode") == "add_node"
        self._flow(fn)

    def test_run_gets_trajectory(self):
        def fn(p, port):
            _click_tool(p, "Point")
            p.click("canvas", position={"x": 300, "y": 360})
            time.sleep(0.1)
            p.evaluate("[...window.sm.entities.values()][0].vx = 2.0")
            _click_run(p)
            t = _get_traj(p)
            assert t is not None, "No trajectory"
            assert t["f"] >= 5
        self._flow(fn)

    def test_collision_bounces(self):
        def fn(p, port):
            _click_tool(p, "Point")
            p.click("canvas", position={"x": 300, "y": 360})
            p.click("canvas", position={"x": 500, "y": 360})
            time.sleep(0.2)
            p.evaluate("""
            var e = [...window.sm.entities.values()];
            e[0].vx = 1.0; e[0].params.radius = 0.1;
            e[1].vx = -1.0; e[1].params.radius = 0.1;
            """)
            # At this version, collision works via batch POST /solve, not streaming
            # Direct API test through the browser:
            result = p.evaluate("""
            async () => {
                var g = window.graphBuilder.build();
                g.system_env.duration = 5;
                var r = await fetch(window.apiClient.base + '/solve', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(g)
                });
                var d = await r.json();
                return {frames: d.t.length, first_vx: d.qd[0][0], last_vx: d.qd[d.qd.length-1][0]};
            }
            """)
            assert result["frames"] > 10
            assert result["first_vx"] > 0 and result["last_vx"] < 0, \
                f"Collision not detected: vx {result['first_vx']} -> {result['last_vx']}"
        self._flow(fn)
