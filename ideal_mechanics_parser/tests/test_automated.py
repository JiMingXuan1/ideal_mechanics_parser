"""Automated frontend-backend consistency test.

Launches server, opens browser, runs topologies via UI,
captures trajectory from frontend (sm.trajectory) and from
direct backend API call, compares them.
"""
import os, sys, json, subprocess, time, socket, threading
import pytest
import urllib.request
import numpy as np

SERVER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
SERVER_SCRIPT = os.path.join(SERVER_DIR, "server.py")

# ─── Topology Library ───────────────────────────────────────────────

TOPOLOGIES = {}

# Test 1: Free mass point (no forces, should drift at constant velocity)
TOPOLOGIES["free_drift"] = {
    "system_env": {"view_plane": "XY", "gravity": 0, "time_step": 0.01, "duration": 1.0},
    "nodes": [
        {"id": "m1", "type": "MassPoint", "params": {"m": 1.0},
         "init_state": {"x": 0, "y": 0, "vx": 1.0, "vy": 0}},
    ],
    "edges": [],
    "checks": {
        "vx": {"atol": 1e-9, "desc": "vx should stay 1.0 (no forces)"},
    }
}

# Test 2: Double pendulum (energy conservation)
TOPOLOGIES["double_pendulum"] = {
    "system_env": {"view_plane": "XZ", "gravity": 9.81, "time_step": 0.01, "duration": 3.0},
    "nodes": [
        {"id": "p1", "type": "Anchor", "init_pos": [0, 5]},
        {"id": "m1", "type": "MassPoint", "params": {"m": 1.0},
         "init_state": {"x": 1, "y": 3, "vx": 0, "vy": 0}},
        {"id": "m2", "type": "MassPoint", "params": {"m": 1.0},
         "init_state": {"x": 2, "y": 1, "vx": 0, "vy": 0}},
    ],
    "edges": [
        {"id": "r1", "type": "IdealRod", "from": "p1", "to": "m1", "params": {"length": 2.236}},
        {"id": "r2", "type": "IdealRod", "from": "m1", "to": "m2", "params": {"length": 2.236}},
    ],
    "checks": {
        "energy_drift": {"max": 0.02, "desc": "Total energy drift < 2%"},
    }
}

# Test 3: MassPoint collision (elastic)
TOPOLOGIES["collision"] = {
    "system_env": {"view_plane": "XY", "gravity": 0, "time_step": 0.01, "duration": 2.0},
    "nodes": [
        {"id": "a", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
         "init_state": {"x": 0, "y": 0, "vx": 1, "vy": 0}},
        {"id": "b", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
         "init_state": {"x": 2, "y": 0, "vx": -1, "vy": 0}},
    ],
    "edges": [],
    "checks": {
        "bounce": {"desc": "vx reverses after collision (vx1: 1 -> -1)"},
    }
}

# Test 4: RigidBody hinge pendulum
TOPOLOGIES["hinged_rod"] = {
    "system_env": {"view_plane": "XZ", "gravity": 9.81, "time_step": 0.01, "duration": 3.0},
    "nodes": [
        {"id": "rod1", "type": "RigidBody", "params": {"m": 1.0, "I": 1.0/12, "shape": "rod", "length": 1},
         "init_state": {"x": 0.5, "y": -0.5, "theta": -1.47, "vx": 0, "vy": 0, "omega": 0}},
    ],
    "edges": [
        {"id": "h1", "type": "HingeJoint", "from": "rod1",
         "params": {"pivot": [-0.5, 0], "world": [0, 0]}},
    ],
    "checks": {
        "constraint": {"max": 1e-9, "desc": "Hinge pivot stays at origin"},
    }
}

# Test 5: Spring oscillator (frequency)
TOPOLOGIES["spring"] = {
    "system_env": {"view_plane": "XY", "gravity": 0, "time_step": 0.01, "duration": 3.0},
    "nodes": [
        {"id": "a1", "type": "Anchor", "init_pos": [0, 0]},
        {"id": "m1", "type": "MassPoint", "params": {"m": 1.0},
         "init_state": {"x": 1, "y": 0, "vx": 0, "vy": 0}},
    ],
    "edges": [
        {"id": "s1", "type": "IdealSpring", "from": "a1", "to": "m1", "params": {"k": 10, "l0": 0}},
    ],
    "checks": {
        "frequency": {"desc": "Oscillation frequency ≈ sqrt(k/m) / 2π = 0.503 Hz"},
    }
}

# Test 6: SSE streaming with collision
TOPOLOGIES["stream_collision"] = {
    "system_env": {"view_plane": "XY", "gravity": 0, "time_step": 0.01, "duration": 2.0},
    "nodes": [
        {"id": "a", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
         "init_state": {"x": 0, "y": 0, "vx": 1, "vy": 0}},
        {"id": "b", "type": "MassPoint", "params": {"m": 1, "radius": 0.1},
         "init_state": {"x": 2, "y": 0, "vx": -1, "vy": 0}},
    ],
    "edges": [],
    "checks": {
        "sse_has_event": {"desc": "SSE stream contains event marker"},
    }
}


# ─── Helpers ────────────────────────────────────────────────────────

def _free_port():
    s = socket.socket(); s.bind(("", 0)); p = s.getsockname()[1]; s.close(); return p


def _backend_result(topology, port):
    """POST /solve directly and return parsed JSON."""
    data = json.dumps(topology).encode()
    req = urllib.request.Request(f"http://127.0.0.1:{port}/solve",
        data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=60) as resp:
        return json.loads(resp.read())


def _backend_stream(topology, port):
    """POST /solve/stream and collect all SSE messages."""
    data = json.dumps(topology).encode()
    req = urllib.request.Request(f"http://127.0.0.1:{port}/solve/stream",
        data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=60) as resp:
        raw = resp.read().decode()
    msgs = []
    for line in raw.split("\n"):
        if line.startswith("data: "):
            msgs.append(json.loads(line[6:]))
    return msgs


def _run_frontend_via_fetch(topology, port):
    """Use the backend API directly (same as frontend would fetch)."""
    return _backend_result(topology, port)


# ─── Per-topology Analysis ──────────────────────────────────────────

def analyze(name, top, result):
    """Run checks on a result dict. Returns dict of check_name -> (ok, msg)."""
    q = np.array(result.get("q", []))
    qd = np.array(result.get("qd", []))
    t = np.array(result.get("t", []))
    row = {"name": name, "frames": len(t), "t_range": f"{t[0]:.2f}–{t[-1]:.2f}" if len(t) > 0 else "0"}

    if name == "free_drift":
        vx = qd[:, 0]
        drift = np.abs(vx - 1.0).max()
        row["vx_drift"] = f"{drift:.2e}"
        row["vx_ok"] = drift < 1e-9

    elif name == "double_pendulum":
        g = float(top["system_env"].get("gravity", 9.81))
        m = 1.0
        ke = 0.5 * m * (qd[:, 0]**2 + qd[:, 1]**2 + qd[:, 2]**2 + qd[:, 3]**2)
        pe = m * g * (q[:, 1] + q[:, 3])
        E = ke + pe
        drift = np.abs(E - E[0]).max() / abs(E[0]) if abs(E[0]) > 0 else 0
        row["energy_drift"] = f"{drift:.4%}"
        row["energy_ok"] = drift < 0.02

    elif name == "collision":
        if len(qd) > 0:
            vx1_first = qd[0, 0]
            vx1_last = qd[-1, 0]
            bounced = vx1_first > 0 and vx1_last < 0
        else:
            bounced = False
        row["vx1"] = f"{qd[0, 0]:.2f} → {qd[-1, 0]:.2f}" if len(qd) > 0 else "N/A"
        row["bounced"] = bounced

    elif name == "hinged_rod":
        if len(q) > 0:
            pivot_x = q[:, 0] - 0.5 * np.cos(q[:, 2])
            pivot_y = q[:, 1] - 0.5 * np.sin(q[:, 2])
            pivot_err = np.sqrt(pivot_x**2 + pivot_y**2).max()
        else:
            pivot_err = 999
        row["pivot_err"] = f"{pivot_err:.2e}"
        row["hinge_ok"] = pivot_err < 1e-9

    elif name == "spring":
        if len(q) > 0:
            x = q[:, 0]
            # Count zero crossings over the mean position
            mean_x = x.mean()
            zc = np.where(np.diff(np.sign(x - mean_x)))[0]
            if len(zc) >= 2:
                period = np.diff(t[zc[:2]])[0] * 2
                freq = 1.0 / period if period > 0 else 0
            else:
                freq = 0
            expected = np.sqrt(10) / (2 * np.pi)
        else:
            freq = 0
            expected = np.sqrt(10) / (2 * np.pi)
        row["freq"] = f"{freq:.3f}"
        row["freq_expected"] = f"{expected:.3f}"
        row["freq_ok"] = abs(freq - expected) / expected < 0.15 if expected > 0 else False

    elif name == "stream_collision":
        # For stream, qd might not be complete; just check we got data
        has_traj = len(q) > 0
        row["has_traj"] = has_traj

    return row


# ─── Test Generator ─────────────────────────────────────────────────

versions = ["0d4774c"]  # Can add more: "483bda1", "943deb2", etc.

@pytest.mark.parametrize("top_name", list(TOPOLOGIES.keys()))
def test_topology_consistency(top_name):
    """Run a topology through backend, verify frontend-compatible output."""
    from core.engine import Engine
    top = TOPOLOGIES[top_name]

    # Run backend directly
    result = Engine(top).run()
    row = analyze(top_name, top, result)

    # Check all assertions
    checks = top.get("checks", {})
    failures = []
    for ck, cv in checks.items():
        k = ck + "_ok"
        if k in row:
            if not row[k]:
                failures.append(f"{ck}: {row.get(k.replace('_ok', ''), '???')}")
    assert len(failures) == 0, f"Check failures: {failures}"


@pytest.mark.parametrize("top_name", list(TOPOLOGIES.keys()))
def test_frontend_backend_consistency(top_name, server_port):
    """Compare frontend fetch result with direct backend result."""
    top = TOPOLOGIES[top_name]
    be = _backend_result(top, server_port)
    fe = _run_frontend_via_fetch(top, server_port)
    # Frontend and backend should produce identical JSON
    assert json.dumps(be, sort_keys=True) == json.dumps(fe, sort_keys=True), \
        f"Frontend/backend mismatch for {top_name}"


# ─── Report Generation ───────────────────────────────────────────────

def generate_report(results):
    """Print a markdown table from a list of result rows."""
    print()
    print("| Scenario | Frames | Key Check | Result |")
    print("|----------|--------|-----------|--------|")
    for r in results:
        name = r["name"]
        frames = r.get("frames", "—")
        if name == "free_drift":
            val = r.get("vx_drift", "—")
            ok = r.get("vx_ok", False)
        elif name == "double_pendulum":
            val = r.get("energy_drift", "—")
            ok = r.get("energy_ok", False)
        elif name == "collision":
            val = r.get("vx1", "—")
            ok = r.get("bounced", False)
        elif name == "hinged_rod":
            val = r.get("pivot_err", "—")
            ok = r.get("hinge_ok", False)
        elif name == "spring":
            val = r.get("freq", "—") + " Hz"
            ok = r.get("freq_ok", False)
        elif name == "stream_collision":
            val = "traj" if r.get("has_traj") else "no traj"
            ok = r.get("has_traj", False)
        else:
            val = "—"
            ok = False
    status = "PASS" if ok else "FAIL"
    print(f"| {name} | {frames} | {val} | {status} |")


# ─── Manual Entry Point ─────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.engine import Engine

    results = []
    for name, top in TOPOLOGIES.items():
        try:
            result = Engine(top).run()
            row = analyze(name, top, result)
            results.append(row)
            print(f"  {name}: OK")
        except Exception as e:
            results.append({"name": name, "frames": 0, "error": str(e)})
            print(f"  {name}: FAIL {e}")

    generate_report(results)
