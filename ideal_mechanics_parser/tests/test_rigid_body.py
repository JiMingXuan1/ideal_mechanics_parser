import pytest
import numpy as np
from core.engine import Engine
from io_handler.parser import parse_json, _validate, VALID_NODE_TYPES, VALID_EDGE_TYPES
from core.exceptions import TopologyError


# ─── Free Rigid Body ────────────────────────────────────────────────────

def test_free_body_conserves_ke():
    """RigidBody with no constraints: KE constant (XY plane, no gravity)."""
    top = make_env("XY", 0)
    top["nodes"].append(body("b1", m=2.0, I=0.5, x=0, y=0, vx=1, vy=2, omega=3))
    r = Engine(top).run()
    qd = np.array(r["qd"])
    ke = 0.5 * 2 * (qd[:, 0]**2 + qd[:, 1]**2) + 0.5 * 0.5 * qd[:, 2]**2
    assert np.abs(ke - ke[0]).max() < 1e-10


def test_free_body_streaming():
    """Streaming a free rigid body: valid chunks with 3 DOFs, body_dofs on first."""
    top = make_env("XY", 0, duration=1.0)
    top["nodes"].append(body("b1", m=1, I=0.1, vx=0.5, omega=1))
    chunks = []
    Engine(top).run_stream(on_chunk=lambda c: chunks.append(c), seg_duration=0.3)
    assert any(c.get("complete") for c in chunks)

    first = chunks[0]
    assert first.get("body_dofs") == [3], "body_dofs missing on first chunk"
    assert first["node_order"] == ["b1"]
    q_arr = np.array(first["q"])
    assert q_arr.shape[1] == 3


# ─── HingeJoint to World Anchor ─────────────────────────────────────────

def test_hinged_rod_constraint_satisfied():
    """HingeJoint constraint (pivot to world) holds within tolerance over full sim."""
    top = hinged_rod_topology(theta=-1.47)
    r = Engine(top).run()
    q = np.array(r["q"])
    t = np.array(r["t"])

    pivot_world = np.column_stack([
        q[:, 0] - 0.5 * np.cos(q[:, 2]),
        q[:, 1] - 0.5 * np.sin(q[:, 2]),
    ])
    pivot_dist = np.linalg.norm(pivot_world, axis=1)
    max_err = np.abs(pivot_dist).max()
    assert max_err < 1e-9, f"Pivot drifted {max_err:.2e} m from origin"


def test_hinged_rod_energy_drift_small():
    """Hinged pendulum: total energy drift < 1% over 5 periods."""
    top = hinged_rod_topology(theta=-1.47)
    r = Engine(top).run()
    q, qd = np.array(r["q"]), np.array(r["qd"])
    m, I = 1.0, 1.0 / 12.0
    ke = 0.5 * m * (qd[:, 0]**2 + qd[:, 1]**2) + 0.5 * I * qd[:, 2]**2
    pe = m * 9.81 * q[:, 1]
    E = ke + pe
    drift = np.abs(E - E[0]).max() / abs(E[0])
    assert drift < 0.01, f"Energy drift {drift:.2%}"


def test_compound_pendulum_period():
    """Small-angle compound pendulum period matches theory within 10%.

    Rod L=1, I_com=1/12, d=0.5  →  T = 2*pi*sqrt((I_com+m*d^2)/(m*g*d)) ≈ 1.64 s
    """
    _, r = _run_hinged_rod(theta0=-np.pi/2 + 0.08, duration=4.0)
    T_est = _estimate_period(r)
    expected = 2 * np.pi * np.sqrt((1/12 + 0.25) / (9.81 * 0.5))
    assert abs(T_est - expected) / expected < 0.1, f"T={T_est:.3f} ≠ {expected:.3f}"


def test_simple_pendulum_limit():
    """RigidBody with I=0 approximates simple pendulum T = 2*pi*sqrt(L/g)."""
    top = make_env("XZ", 9.81, duration=4.0)
    L = 1.5
    theta0 = -np.pi/2 + 0.05
    top["nodes"].append(body("b", m=1.0, I=0.0,
                             x=L*np.cos(theta0), y=L*np.sin(theta0),
                             theta=theta0))
    top["edges"].append(hinge("b", pivot=[-L, 0], world=[0, 0]))
    r = Engine(top).run()

    q = np.array(r["q"])
    eq = -np.pi / 2
    t = np.array(r["t"])
    cross = np.where(np.diff(np.sign(q[:, 2] - eq)))[0]
    assert len(cross) >= 2
    T_est = (t[cross[1]] - t[cross[0]]) * 2
    T_exp = 2 * np.pi * np.sqrt(L / 9.81)
    assert abs(T_est - T_exp) / T_exp < 0.1, f"T={T_est:.3f} ≠ {T_exp:.3f}"


def test_hinged_to_anchor_node():
    """HingeJoint can connect to an Anchor node via to_id (not world)."""
    top = make_env("XZ", 9.81, duration=2.0)
    top["nodes"].append({"id": "a1", "type": "Anchor", "init_pos": [0, 0]})
    theta0 = -np.pi/2 + 0.1
    top["nodes"].append(body("b1", m=1.0, I=1/12,
                             x=0.5*np.cos(theta0), y=0.5*np.sin(theta0),
                             theta=theta0))
    top["edges"].append(hinge("b1", to_id="a1", pivot=[-0.5, 0]))
    r = Engine(top).run()

    q = np.array(r["q"])
    pivot_world = np.column_stack([
        q[:, 0] - 0.5 * np.cos(q[:, 2]),
        q[:, 1] - 0.5 * np.sin(q[:, 2]),
    ])
    assert np.linalg.norm(pivot_world, axis=1).max() < 1e-9


# ─── Mixed MassPoint + RigidBody ────────────────────────────────────────

def test_body_hinged_to_mass_point():
    """RigidBody COM hinged to a MassPoint: constraint stays satisfied.

    Use matching initial velocities to avoid velocity-level drift.
    """
    top = make_env("XY", 0, duration=2.0)
    vx0, vy0 = 0.5, 0.3
    top["nodes"].append({"id": "mp", "type": "MassPoint",
                         "params": {"m": 0.5},
                         "init_state": {"x": 0, "y": 0, "vx": vx0, "vy": vy0}})
    top["nodes"].append(body("rb", m=1.0, I=0.1,
                             x=0.0, y=0.0, theta=0.3,
                             vx=vx0, vy=vy0, omega=0.0))
    top["edges"].append(hinge("rb", to_id="mp", pivot=[0, 0]))
    r = Engine(top).run()

    q = np.array(r["q"])
    mp_pos = q[:, :2]
    rb_com = q[:, 2:4]
    conn_err = np.linalg.norm(rb_com - mp_pos, axis=1)
    assert conn_err.max() < 1e-9, f"Hinge constraint violated: {conn_err.max():.2e}"


def test_mixed_system_energy_conservation():
    """MassPoint + RigidBody with no constraints: total KE conserved (XY)."""
    top = make_env("XY", 0, duration=1.0)
    top["nodes"].append({"id": "mp", "type": "MassPoint",
                         "params": {"m": 0.5},
                         "init_state": {"x": 0, "y": 0, "vx": 1, "vy": 0}})
    top["nodes"].append(body("rb", m=1.0, I=0.2,
                             x=0, y=0, theta=0.5, vx=0, vy=1, omega=2))
    r = Engine(top).run()

    qd = np.array(r["qd"])
    ke_mp = 0.5 * 0.5 * (qd[:, 0]**2 + qd[:, 1]**2)
    ke_rb = 0.5 * 1.0 * (qd[:, 2]**2 + qd[:, 3]**2) + 0.5 * 0.2 * qd[:, 4]**2
    ke_tot = ke_mp + ke_rb
    assert np.abs(ke_tot - ke_tot[0]).max() < 1e-10


# ─── RigidBody ↔ RigidBody Hinge ──────────────────────────────────────

def test_two_body_hinge_conserves_momentum():
    """Two rigid bodies hinged together: linear momentum conserved (XY, no gravity)."""
    top = make_env("XY", 0, duration=1.0)
    top["nodes"].append(body("b1", m=1.0, I=0.1,
                             x=-0.5, y=0, vx=1.0, vy=0, omega=0))
    top["nodes"].append(body("b2", m=2.0, I=0.2,
                             x=0.5, y=0, vx=-0.5, vy=0, omega=0))
    top["edges"].append({
        "id": "h", "type": "HingeJoint", "from": "b1", "to": "b2",
        "params": {"pivot": [0.5, 0], "pivot_b": [-0.5, 0]},
    })
    r = Engine(top).run()

    qd = np.array(r["qd"])
    p_tot = 1.0 * qd[:, 0:2] + 2.0 * qd[:, 3:5]  # no theta in linear momentum
    drift = np.abs(p_tot - p_tot[0]).max()
    assert drift.max() < 1e-10, f"Linear momentum drift: {drift.max():.2e}"


def test_two_body_hinge_constraint():
    """Two hinged bodies: the connection point stays coincident.

    Velocities are chosen so the pivot-point velocities match:
        v_pivot_i = v_com_i + omega_i × r_i
    """
    top = make_env("XY", 0, duration=1.0)
    # b1 COM at (-0.5, 0), pivot at offset [0.5, 0]
    # b2 COM at (0.5, 0), pivot at offset [-0.5, 0]
    # Initially both pivots at (0, 0)
    # Pivot velocity: v_com + omega × r
    #   v_p1 = v1 + cross(omega1, [0.5,0]) = v1 + [0, 0.5*omega1]
    #   v_p2 = v2 + cross(omega2, [-0.5,0]) = v2 + [0, -0.5*omega2]
    # Choose v1, v2 such that v_p1 = v_p2:
    #   vx1 = 0.5, vy1 = 0.3, omega1 = 0.2 → v_p1 = [0.5, 0.3+0.1] = [0.5, 0.4]
    #   v_p2 = [vx2, vy2 - 0.5*omega2] = [0.5, 0.4]
    #   Let omega2 = 0.1 → vy2 = 0.4 + 0.05 = 0.45, vx2 = 0.5
    top["nodes"].append(body("b1", m=1.0, I=0.1,
                             x=-0.5, y=0, vx=0.5, vy=0.3, omega=0.2))
    top["nodes"].append(body("b2", m=2.0, I=0.2,
                             x=0.5, y=0, vx=0.5, vy=0.45, omega=0.1))
    top["edges"].append({
        "id": "h", "type": "HingeJoint", "from": "b1", "to": "b2",
        "params": {"pivot": [0.5, 0], "pivot_b": [-0.5, 0]},
    })
    r = Engine(top).run()

    q = np.array(r["q"])
    p1 = np.column_stack([q[:, 0] + 0.5 * np.cos(q[:, 2]),
                           q[:, 1] + 0.5 * np.sin(q[:, 2])])
    p2 = np.column_stack([q[:, 3] - 0.5 * np.cos(q[:, 5]),
                           q[:, 4] - 0.5 * np.sin(q[:, 5])])
    err = np.linalg.norm(p1 - p2, axis=1)
    assert err.max() < 1e-9, f"Hinge misalignment: {err.max():.2e}"


# ─── Angular Momentum Conservation ────────────────────────────────────

def test_free_body_angular_momentum_conserved():
    """Free rigid body: spin angular momentum I*ω is constant (no torques)."""
    top = make_env("XY", 0, duration=2.0)
    top["nodes"].append(body("b", m=2.0, I=0.5, vx=1, vy=2, omega=3))
    r = Engine(top).run()
    qd = np.array(r["qd"])
    L_spin = 0.5 * qd[:, 2]  # I * omega
    assert np.abs(L_spin - L_spin[0]).max() < 1e-10


def test_free_body_linear_momentum_conserved():
    """Free rigid body: linear momentum m*v is constant."""
    top = make_env("XY", 0, duration=2.0)
    top["nodes"].append(body("b", m=2.0, I=0.5, vx=1, vy=2, omega=3))
    r = Engine(top).run()
    qd = np.array(r["qd"])
    p = 2.0 * qd[:, :2]  # m * v
    assert np.abs(p - p[0]).max() < 1e-10


def test_hinged_rod_angular_momentum_xy():
    """Hinged rod in XY plane (no gravity): angular momentum about hinge conserved.

    Need velocity-level constraint consistency (f_dot = J·qd = 0) to avoid
    Baumgarte artificial torque.  COM velocity follows from θ̇: v = ω × r.
    """
    top = make_env("XY", 0, duration=2.0)
    d, m, I = 0.5, 1.0, 1.0 / 12.0
    theta0, omega0 = 0.3, 2.0
    ct, st = np.cos(theta0), np.sin(theta0)
    top["nodes"].append(body("b", m=m, I=I,
                             x=d*ct, y=d*st, theta=theta0,
                             vx=-d*st*omega0, vy=d*ct*omega0,
                             omega=omega0))
    top["edges"].append(hinge("b", pivot=[-d, 0], world=[0, 0]))
    r = Engine(top).run()

    q, qd = np.array(r["q"]), np.array(r["qd"])
    omega, vx, vy = qd[:, 2], qd[:, 0], qd[:, 1]
    x, y = q[:, 0], q[:, 1]

    L_orb = m * (x * vy - y * vx)
    L_tot = I * omega + L_orb
    drift = np.abs(L_tot - L_tot[0]).max()
    assert drift < 1e-10, f"Angular momentum drift: {drift:.2e}"


def test_two_body_hinge_angular_momentum():
    """Two hinged bodies in XY (no gravity): total angular momentum conserved.

    L_tot = Σ (I_i*ω_i + m_i * r_i × v_i) where r_i is about the system COM.
    """
    top = make_env("XY", 0, duration=1.0)
    m1, I1 = 1.0, 0.1
    m2, I2 = 2.0, 0.2
    top["nodes"].append(body("b1", m=m1, I=I1,
                             x=-0.5, y=0, vx=0.5, vy=0.3, omega=0.2))
    top["nodes"].append(body("b2", m=m2, I=I2,
                             x=0.5, y=0, vx=0.5, vy=0.45, omega=0.1))
    top["edges"].append({
        "id": "h", "type": "HingeJoint", "from": "b1", "to": "b2",
        "params": {"pivot": [0.5, 0], "pivot_b": [-0.5, 0]},
    })
    r = Engine(top).run()

    q, qd = np.array(r["q"]), np.array(r["qd"])
    x1, y1, t1 = q[:, 0], q[:, 1], q[:, 2]
    x2, y2, t2 = q[:, 3], q[:, 4], q[:, 5]
    vx1, vy1, w1 = qd[:, 0], qd[:, 1], qd[:, 2]
    vx2, vy2, w2 = qd[:, 3], qd[:, 4], qd[:, 5]

    # System COM
    M = m1 + m2
    X_cm = (m1 * x1 + m2 * x2) / M
    Y_cm = (m1 * y1 + m2 * y2) / M

    # Position relative to COM
    rx1, ry1 = x1 - X_cm, y1 - Y_cm
    rx2, ry2 = x2 - X_cm, y2 - Y_cm

    # Velocity relative to COM
    Vx_cm = (m1 * vx1 + m2 * vx2) / M
    Vy_cm = (m1 * vy1 + m2 * vy2) / M
    rvx1, rvy1 = vx1 - Vx_cm, vy1 - Vy_cm
    rvx2, rvy2 = vx2 - Vx_cm, vy2 - Vy_cm

    # L = Σ (I_i*ω_i + m_i * r_i × v_i)
    L1 = I1 * w1 + m1 * (rx1 * rvy1 - ry1 * rvx1)
    L2 = I2 * w2 + m2 * (rx2 * rvy2 - ry2 * rvx2)
    L_tot = L1 + L2
    drift = np.abs(L_tot - L_tot[0]).max()
    assert drift < 1e-10, f"Total angular momentum drift: {drift:.2e}"


# ─── Response format ──────────────────────────────────────────────────

def test_batch_response_has_body_dofs():
    """run() response includes body_dofs array.

    node_order puts MassPoints before RigidBodies regardless of JSON order.
    """
    top = make_env("XY", 0)
    top["nodes"].append(body("b1", x=1, y=2))
    top["nodes"].append({"id": "mp", "type": "MassPoint",
                         "params": {"m": 1}, "init_state": {"x": 0, "y": 0}})
    r = Engine(top).run()
    assert "body_dofs" in r
    # node_order = [mp, b1], body_dofs = [2, 3]
    assert r["body_dofs"] == [2, 3]


# ─── Validation ───────────────────────────────────────────────────────

def test_parser_validates_new_types():
    assert "RigidBody" in VALID_NODE_TYPES
    assert "HingeJoint" in VALID_EDGE_TYPES


def test_rigid_body_passes_validation():
    top = make_env("XY", 0)
    top["nodes"].append(body("b1"))
    _validate(top)


def test_hinge_joint_passes_validation():
    top = make_env("XY", 0)
    top["nodes"].append(body("b1"))
    top["edges"].append(hinge("b1", world=[0, 0]))
    _validate(top)


# ─── Edge Cases ───────────────────────────────────────────────────────

def test_zero_inertia_body():
    """RigidBody with I=0 behaves like a mass point in XY plane."""
    top = make_env("XY", 0, duration=1.0)
    top["nodes"].append(body("b", m=1.0, I=0.0, vx=2.0, vy=0, omega=5.0))
    r = Engine(top).run()

    qd = np.array(r["qd"])
    # vx, vy should stay constant
    assert np.abs(qd[:, 0] - 2.0).max() < 1e-10
    # omega stays constant even with I=0 (equation degenerates to 0=0)
    ke = 0.5 * 1.0 * (qd[:, 0]**2 + qd[:, 1]**2)
    assert np.abs(ke - ke[0]).max() < 1e-10


def test_gravity_on_rigid_body():
    """In XZ plane, rigid body accelerates downward with g."""
    top = make_env("XZ", 9.81, duration=0.5)
    top["nodes"].append(body("b", m=1.0, I=0.1, x=0, y=5.0))
    r = Engine(top).run()
    q = np.array(r["q"])
    dy = q[-1, 1] - q[0, 1]
    assert dy < 0, "Body should fall in XZ plane"
    # free fall: y ≈ y0 - 0.5*g*t²
    y_expected = 5.0 - 0.5 * 9.81 * 0.5**2
    assert abs(q[-1, 1] - y_expected) < 0.01, f"y={q[-1,1]:.3f} ≠ {y_expected:.3f}"


def test_no_gravity_in_xy():
    """In XY plane, rigid body should not accelerate due to gravity."""
    top = make_env("XY", 9.81, duration=0.5)
    top["nodes"].append(body("b", m=1.0, I=0.1, x=0, y=5.0))
    r = Engine(top).run()
    q = np.array(r["q"])
    dy = abs(q[-1, 1] - q[0, 1])
    assert dy < 1e-10, "Body should not move in XY plane"


# ─── N-R Projection ───────────────────────────────────────────────────

def test_projection_fixes_hinged_initial_state():
    """HingeJoint initial state that violates constraint gets corrected by N-R."""
    top = make_env("XZ", 9.81, duration=0.1)
    # Intentionally bad init: COM at (0,0) but pivot offset requires x=0.5*cos(theta)
    top["nodes"].append(body("b", m=1.0, I=1/12,
                             x=0.0, y=0.0, theta=0.0))
    top["edges"].append(hinge("b", pivot=[-0.5, 0], world=[0, 0]))
    r = Engine(top).run()

    q = np.array(r["q"])
    x_proj, y_proj, theta_proj = q[0]
    # After projection: x must be 0.5*cos(theta), y must be 0.5*sin(theta)
    err = abs(x_proj - 0.5 * np.cos(theta_proj)) + abs(y_proj - 0.5 * np.sin(theta_proj))
    assert err < 1e-12, f"Projected state violates constraint: err={err:.2e}"


def test_projection_on_manifold_unchanged():
    """Initial state already on constraint manifold should not change."""
    top = make_env("XZ", 9.81, duration=0.1)
    theta0 = 0.3
    x0, y0 = 0.5 * np.cos(theta0), 0.5 * np.sin(theta0)
    top["nodes"].append(body("b", m=1.0, I=1/12,
                             x=x0, y=y0, theta=theta0))
    top["edges"].append(hinge("b", pivot=[-0.5, 0], world=[0, 0]))
    r = Engine(top).run()

    q = np.array(r["q"])
    assert abs(q[0, 0] - x0) < 1e-12
    assert abs(q[0, 1] - y0) < 1e-12
    assert abs(q[0, 2] - theta0) < 1e-12


# ─── Helpers ──────────────────────────────────────────────────────────

def make_env(plane="XY", gravity=0, duration=2.0):
    return {"system_env": {"view_plane": plane, "gravity": gravity,
                           "duration": duration, "time_step": 0.01},
            "nodes": [], "edges": []}


def body(bid, m=1.0, I=1/12, x=0, y=0, theta=0, vx=0, vy=0, omega=0):
    return {"id": bid, "type": "RigidBody",
            "params": {"m": m, "I": I},
            "init_state": {"x": x, "y": y, "theta": theta,
                           "vx": vx, "vy": vy, "omega": omega}}


def hinge(frm, pivot=None, world=None, to_id=None):
    e = {"id": f"h_{frm}", "type": "HingeJoint", "from": frm}
    if to_id:
        e["to"] = to_id
    e["params"] = {}
    if pivot is not None:
        e["params"]["pivot"] = pivot
    if world is not None:
        e["params"]["world"] = world
    return e


def hinged_rod_topology(theta=-1.47):
    top = make_env("XZ", 9.81, duration=3.0)
    top["nodes"].append(body("rod1", m=1.0, I=1/12,
                             x=0.5*np.cos(theta), y=0.5*np.sin(theta),
                             theta=theta))
    top["edges"].append(hinge("rod1", pivot=[-0.5, 0], world=[0, 0]))
    return top


def _run_hinged_rod(theta0=-np.pi/2 + 0.08, duration=4.0):
    top = hinged_rod_topology(theta=theta0)
    top["system_env"]["duration"] = duration
    r = Engine(top).run()
    return top, r


def _estimate_period(result):
    q = np.array(result["q"])
    t = np.array(result["t"])
    eq = -np.pi / 2
    cross = np.where(np.diff(np.sign(q[:, 2] - eq)))[0]
    if len(cross) < 2:
        return None
    return (t[cross[1]] - t[cross[0]]) * 2
