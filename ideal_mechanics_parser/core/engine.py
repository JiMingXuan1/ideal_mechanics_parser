import sympy as sp
import numpy as np
from scipy.integrate import solve_ivp
from .symbols import SymbolManager
from .energy import assemble_energy
from .constraints import harvest_constraints
from .projection import project_initial_state, project_initial_velocity
from .numerical import NumericalIntegrator
from entities.mass_point import MassPoint
from entities.rigid_body import RigidBody
from entities.ideal_rod import IdealRod
from entities.ideal_spring import IdealSpring
from entities.smooth_rail import SmoothRail
from entities.fixed_coordinate import FixedCoordinate
from entities.linear_relation import LinearRelation
from entities.distance_sum import DistanceSum
from entities.angle_constraint import AngleConstraint
from entities.hinge_joint import HingeJoint
from entities.soft_rope import SoftRope
from safety.sympify_sandbox import safe_sympify


def _is_dynamic(node):
    return node is not None and hasattr(node, "idx") and node.idx is not None


def _node_init_pos(node):
    """Initial (x, y) of a node, or (None, None) if not resolvable."""
    if hasattr(node, "init_state") and node.init_state:
        return (float(node.init_state.get("x", 0.0)),
                float(node.init_state.get("y", 0.0)))
    xs, ys = getattr(node, "x_sym", None), getattr(node, "y_sym", None)
    if xs is None or ys is None:
        return None, None
    try:
        if (hasattr(xs, "has") and xs.has(sp.Symbol("t"))) or \
           (hasattr(ys, "has") and ys.has(sp.Symbol("t"))):
            return None, None
        return float(xs), float(ys)
    except Exception:
        return None, None


def _point_segment_closest(px, py, sx, sy, ex, ey):
    """Closest point (cx, cy) on segment SE to point P, plus distance."""
    exx, eyy = ex - sx, ey - sy
    t = ((px - sx) * exx + (py - sy) * eyy) / (exx*exx + eyy*eyy + 1e-30)
    t = max(0.0, min(1.0, t))
    cx = sx + t * exx
    cy = sy + t * eyy
    dx = px - cx
    dy = py - cy
    return (cx, cy), np.sqrt(dx * dx + dy * dy)


def _segments_intersect(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y):
    """True if segments AB and CD intersect (proper crossing or touching)."""
    def orient(px, py, qx, qy, rx, ry):
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)

    def on_seg(px, py, sx, sy, ex, ey):
        return (min(sx, ex) - 1e-12 <= px <= max(sx, ex) + 1e-12 and
                min(sy, ey) - 1e-12 <= py <= max(sy, ey) + 1e-12)

    d1 = orient(b1x, b1y, b2x, b2y, a1x, a1y)
    d2 = orient(b1x, b1y, b2x, b2y, a2x, a2y)
    d3 = orient(a1x, a1y, a2x, a2y, b1x, b1y)
    d4 = orient(a1x, a1y, a2x, a2y, b2x, b2y)

    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    if d1 == 0 and on_seg(a1x, a1y, b1x, b1y, b2x, b2y):
        return True
    if d2 == 0 and on_seg(a2x, a2y, b1x, b1y, b2x, b2y):
        return True
    if d3 == 0 and on_seg(b1x, b1y, a1x, a1y, a2x, a2y):
        return True
    if d4 == 0 and on_seg(b2x, b2y, a1x, a1y, a2x, a2y):
        return True
    return False


def _segment_intersection(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y):
    """Intersection point of two segments that are known to intersect."""
    denom = (a1x - a2x) * (b1y - b2y) - (a1y - a2y) * (b1x - b2x)
    if abs(denom) < 1e-30:
        return ((a1x + a2x + b1x + b2x) / 4.0, (a1y + a2y + b1y + b2y) / 4.0)
    t = ((a1x - b1x) * (b1y - b2y) - (a1y - b1y) * (b1x - b2x)) / denom
    return (a1x + t * (a2x - a1x), a1y + t * (a2y - a1y))


def _segment_closest_points(a1, a2, b1, b2):
    """Closest point pair (pa on AB, pb on CD) and their distance.

    Handles intersecting segments correctly: the distance is 0 and the
    contact point is the intersection (the old endpoint-only computation
    returned a positive distance for crossing segments, so rod-rod
    collisions in an X shape were never detected).
    """
    a1x, a1y = a1
    a2x, a2y = a2
    b1x, b1y = b1
    b2x, b2y = b2
    if _segments_intersect(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y):
        ip = _segment_intersection(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y)
        return ip, ip, 0.0

    best_d = None
    best = None
    for pa in (a1, a2):
        pb, d = _point_segment_closest(pa[0], pa[1], b1x, b1y, b2x, b2y)
        if best_d is None or d < best_d:
            best_d, best = d, (pa, pb)
    for pb in (b1, b2):
        pa, d = _point_segment_closest(pb[0], pb[1], a1x, a1y, a2x, a2y)
        if d < best_d:
            best_d, best = d, (pa, pb)
    return best[0], best[1], best_d


def _edge_initial_attachment(node, pivot):
    """Initial world position of an edge's attachment point on a node.

    Pivot-aware for rigid bodies (init_state x/y/theta + local pivot),
    mirroring the symbolic _world_pivot in constraints.py but numeric.
    Returns (None, None) if the node's position is not resolvable.
    """
    if hasattr(node, "theta_sym") and node.theta_sym is not None:
        x = float(node.init_state.get("x", 0.0))
        y = float(node.init_state.get("y", 0.0))
        th = float(node.init_state.get("theta", 0.0))
        u, v = pivot if pivot is not None else (0.0, 0.0)
        ct, st = np.cos(th), np.sin(th)
        return (x + u * ct - v * st, y + u * st + v * ct)
    return _node_init_pos(node)


class Engine:
    CONSTRAINT_TYPES = {"IdealRod", "SmoothRail", "FixedCoordinate", "LinearRelation",
                        "DistanceSum", "AngleConstraint", "HingeJoint"}

    # Post-impulse normal speed below this is treated as resting contact
    # (converted to a bilateral distance constraint).
    REST_EPS = 1e-4
    # A resting-contact multiplier above this value means the contact is in
    # tension (pulling instead of pushing) and must be released.
    TENSION_EPS = 1e-4

    _NODE_TYPES = {
        "MassPoint": MassPoint,
        "RigidBody": RigidBody,
    }

    _EDGE_TYPES = {
        "IdealRod": IdealRod,
        "IdealSpring": IdealSpring,
        "SmoothRail": SmoothRail,
        "FixedCoordinate": FixedCoordinate,
        "LinearRelation": LinearRelation,
        "DistanceSum": DistanceSum,
        "AngleConstraint": AngleConstraint,
        "HingeJoint": HingeJoint,
        "SoftRope": SoftRope,
    }

    def __init__(self, topology):
        self.topology = topology
        self.sm = SymbolManager()
        self.points = []
        self.rigid_bodies = []
        self.edges = []
        self.node_map = {}

    def run(self):
        self._step1_instantiate()
        self._step2_project()
        self._step3_energy()
        self._step4_constraints()
        result = self._step5_integrate()
        return result

    def _needs_events(self):
        for n in self.topology.get("nodes", []):
            if float(n.get("params", {}).get("radius", 0)) > 0:
                return True
        for e in self.topology.get("edges", []):
            if e.get("type") == "SoftRope":
                return True
        return False

    def run_all(self):
        """Batch entry point: route to run_events (collisions/SoftRope) or run()."""
        if not self._needs_events():
            return self.run()

        chunks = []
        self.run_events(on_chunk=lambda c: chunks.append(c))
        result = {"t": [], "q": [], "qd": [], "node_order": [], "body_dofs": []}
        for c in chunks:
            if "error" in c:
                raise RuntimeError(c["error"])
            if "t" in c:
                if not result["t"]:
                    result["node_order"] = c.get("node_order", [])
                    result["body_dofs"] = c.get("body_dofs", [])
                result["t"].extend(c["t"])
                result["q"].extend(c["q"])
                if c.get("qd"):
                    result["qd"].extend(c["qd"])
        return result

    def _step1_instantiate(self):
        for node in self.topology["nodes"]:
            ntype = node["type"]
            if ntype == "MassPoint":
                p = MassPoint(node["id"], node.get("init_state"), node.get("params"))
                p.m = float(node.get("params", {}).get("m", 1.0))
                p.radius = float(node.get("params", {}).get("radius", 0.0))
                p._fx_expr = node.get("params", {}).get("external_force_x_expr")
                p._fy_expr = node.get("params", {}).get("external_force_y_expr")
                self.sm.add_point(p)
                self.points.append(p)
            elif ntype == "RigidBody":
                b = RigidBody(node["id"], node.get("init_state"), node.get("params"))
                b.m = float(node.get("params", {}).get("m", 1.0))
                b.radius = float(node.get("params", {}).get("radius", 0.0))
                # Auto-compute I from shape if not explicitly provided
                explicit_I = node.get("params", {}).get("I")
                if explicit_I is not None:
                    b.I = float(explicit_I)
                else:
                    shape = b.params.get("shape", "rect")
                    L = float(b.params.get("length", 2.0))
                    W = float(b.params.get("width", 0.5))
                    if shape == "rod":
                        b.I = b.m * L * L / 12.0
                    else:
                        b.I = b.m * (L * L + W * W) / 12.0
                self.sm.add_rigid_body(b)
                self.rigid_bodies.append(b)

        for edge_data in self.topology["edges"]:
            cls = self._EDGE_TYPES.get(edge_data["type"])
            if cls is None:
                continue
            e = cls(edge_data["id"], edge_data.get("from"), edge_data.get("to"), edge_data.get("params"))
            # Copy pivot fields from params to edge object for constraint resolution
            params = edge_data.get("params") or {}
            if "from_pivot" in params:
                e.from_pivot = params["from_pivot"]
            if "to_pivot" in params:
                e.to_pivot = params["to_pivot"]
            # HingeJoint uses "pivot" / "pivot_b" as aliases for from_pivot / to_pivot
            if e.type == "HingeJoint":
                if "pivot" in params and "from_pivot" not in params:
                    e.from_pivot = params["pivot"]
                if "pivot_b" in params and "to_pivot" not in params:
                    e.to_pivot = params["pivot_b"]
            self.edges.append(e)

        node_map = {}
        for p in self.points:
            node_map[p.id] = p
        for b in self.rigid_bodies:
            node_map[b.id] = b
        for n in self.topology["nodes"]:
            if n["type"] == "Anchor" and n["id"] not in node_map:
                params = n.get("params", {})
                x_expr = params.get("x_expr")
                y_expr = params.get("y_expr")
                if x_expr or y_expr:
                    node_map[n["id"]] = MovingAnchor(n["id"], x_expr, y_expr,
                                                     n.get("init_pos", [0, 0]), self.sm)
                else:
                    node_map[n["id"]] = AnchorLike(n["id"], n.get("init_pos", [0, 0]))
        self.node_map = node_map

        for e in self.edges:
            e.from_node = node_map.get(e.from_id)
            if e.to_id:
                e.to_node = node_map.get(e.to_id)

            if hasattr(e, "via_id") and e.via_id:
                e.via_node = node_map.get(e.via_id)

            if e.from_node is None:
                raise ValueError(f"Edge {e.id}: from_node '{e.from_id}' not found")

            # Backfill rod/rope length and spring rest length from the actual
            # initial geometry when the caller did not specify them.  This
            # keeps the GUI (which creates edges before knowing the on-canvas
            # distance) and JSON API callers from getting a default length
            # that makes the projection jump the nodes on the first run.
            if e.type in ("IdealRod", "SoftRope") and "length" not in e.params:
                a, b = e.from_node, e.to_node
                if a is not None and b is not None:
                    ax, ay = _edge_initial_attachment(
                        a, getattr(e, "from_pivot", None))
                    bx, by = _edge_initial_attachment(
                        b, getattr(e, "to_pivot", None))
                    if ax is not None and bx is not None:
                        e.length = np.hypot(ax - bx, ay - by)
            elif e.type == "IdealSpring" and "l0" not in e.params:
                a, b = e.from_node, e.to_node
                if a is not None and b is not None:
                    ax, ay = _edge_initial_attachment(
                        a, getattr(e, "from_pivot", None))
                    bx, by = _edge_initial_attachment(
                        b, getattr(e, "to_pivot", None))
                    if ax is not None and bx is not None:
                        e.l0 = np.hypot(ax - bx, ay - by)

            # SmoothRail: when 'from' is a static anchor, the rail constrains
            # the dynamic endpoint of this edge, not an arbitrary first point.
            if e.type == "SmoothRail" and not _is_dynamic(e.from_node):
                target = None
                for cand in (e.to_node, e.from_node):
                    if _is_dynamic(cand):
                        target = cand
                        break
                if target is not None:
                    e._rail_target = target

            # SoftRope: a rope starting at (or beyond) its length is taut from
            # the start and constrains immediately.
            if e.type == "SoftRope" and not getattr(e, "_tight", False):
                a, b = e.from_node, e.to_node
                if a is not None and b is not None:
                    ax, ay = _node_init_pos(a)
                    bx, by = _node_init_pos(b)
                    if ax is not None and bx is not None:
                        d0 = np.hypot(ax - bx, ay - by)
                        if d0 >= e.length - 1e-9:
                            e._tight = True

    def _step2_project(self):
        q0 = self.sm.get_q0(self.points, self.rigid_bodies)
        qd0 = self.sm.get_qd0(self.points, self.rigid_bodies)

        if self._has_constraints():
            # Validate: constraint must involve at least one dynamic body
            for e in self.edges:
                if e.type in ("SmoothRail",):
                    # SmoothRail: from/to can be anchors defining the track.
                    # The constraint is applied to the MassPoint automatically.
                    pass
                elif e.type in ("FixedCoordinate", "LinearRelation"):
                    if isinstance(e.from_node, AnchorLike):
                        raise ValueError(
                            f"Edge '{e.id}' ({e.type}): 'from' node '{e.from_id}' is an Anchor. "
                            f"Connect 'from' to a MassPoint or RigidBody."
                        )
            L, _ = assemble_energy(self.points, self.edges, self.topology["system_env"],
                                   self.sm, self.rigid_bodies)
            holonomic = harvest_constraints(self.edges, self.sm)
            q = self.sm.q

            if holonomic:
                f_sym = sp.Matrix(holonomic)
                J_sym = f_sym.jacobian(q)
                has_t = any(f.has(self.sm.t) for f in holonomic)
                # Build context for better error messages
                ctx = {
                    "expressions": [str(f) for f in holonomic],
                    "n_constraints": len(holonomic),
                }
                if has_t:
                    proj_args = tuple(q) + (self.sm.t,)
                    f_func = sp.lambdify(proj_args, f_sym, modules="numpy")
                    J_func = sp.lambdify(proj_args, J_sym, modules="numpy")
                    q0 = project_initial_state(q0, qd0, f_func, J_func, extra_args=(0.0,), context=ctx)
                    # Numeric central difference for ∂f/∂t at (q0, t=0)
                    # (symbolic Derivative cannot be lambdified by NumPyPrinter).
                    h = 1e-6
                    def f_t_func(*args):
                        q_vals = list(args[:-1])
                        t_val = args[-1]
                        f_p = np.asarray(f_func(*(q_vals + [t_val + h]))).ravel()
                        f_m = np.asarray(f_func(*(q_vals + [t_val - h]))).ravel()
                        return (f_p - f_m) / (2.0 * h)
                    qd0 = project_initial_velocity(q0, qd0, J_func, f_t_func, extra_args=(0.0,))
                else:
                    f_func = sp.lambdify(tuple(q), f_sym, modules="numpy")
                    J_func = sp.lambdify(tuple(q), J_sym, modules="numpy")
                    q0 = project_initial_state(q0, qd0, f_func, J_func, context=ctx)
                    qd0 = project_initial_velocity(q0, qd0, J_func)

        self.q0_projected = q0
        self.qd0 = qd0

    def _has_constraints(self):
        return any(e.type in self.CONSTRAINT_TYPES for e in self.edges)

    def _step3_energy(self):
        env = self.topology["system_env"]
        self.T, self.V = assemble_energy(self.points, self.edges, env, self.sm, self.rigid_bodies)
        # N-body gravity is assembled inside assemble_energy when env.gravitation.enabled

    def _step4_constraints(self):
        self.holonomic = harvest_constraints(self.edges, self.sm)

    def _build_external_forces(self):
        """Build list of (coord_index, sympy_expr) for external generalized forces."""
        forces = []
        env = self.topology.get("system_env", {})
        g = float(env.get("gravity", 9.81))
        for p in self.points:
            fx = getattr(p, "_fx_expr", None)
            fy = getattr(p, "_fy_expr", None)
            local_vars = {"t": self.sm.t, "m": p.m, "g": g}
            if fx:
                expr = safe_sympify(fx, local_vars)
                forces.append((self._body_dof_index(p), expr))
            if fy:
                expr = safe_sympify(fy, local_vars)
                forces.append((self._body_dof_index(p) + 1, expr))
        return forces

    def _step5_integrate(self):
        L = self.T - self.V
        q = self.sm.q
        qd = self.sm.qd
        env = self.topology["system_env"]

        ext_forces = self._build_external_forces()
        integrator = NumericalIntegrator(L, q, qd, self.holonomic, self.sm, external_forces=ext_forces)

        duration = float(env.get("duration", 10.0))
        time_step = float(env.get("time_step", 0.01))
        t_eval = np.arange(0.0, duration + time_step, time_step)

        t, q_traj, qd_traj = integrator.integrate(self.q0_projected, self.qd0, t_eval)
        node_order = [p.id for p in self.points] + [b.id for b in self.rigid_bodies]
        body_dofs = [2] * len(self.points) + [3] * len(self.rigid_bodies)
        return {
            "t": t.tolist(),
            "q": q_traj.tolist(),
            "qd": qd_traj.tolist(),
            "node_order": node_order,
            "body_dofs": body_dofs,
        }

    def run_stream(self, on_chunk, seg_duration=0.5, _skip_init=False):
        if not _skip_init:
            self._step1_instantiate()
            self._step2_project()
            self._step3_energy()
            self._step4_constraints()
        L = self.T - self.V
        q = self.sm.q
        qd = self.sm.qd
        env = self.topology["system_env"]
        ext_forces = self._build_external_forces()
        integrator = NumericalIntegrator(L, q, qd, self.holonomic, self.sm, external_forces=ext_forces)

        dt = float(env.get("time_step", 0.01))
        duration = float(env.get("duration", 10.0))
        t_full = np.arange(0.0, duration + dt, dt)
        state = np.concatenate([self.q0_projected, self.qd0])
        nq = self.sm.nq
        node_order = [p.id for p in self.points] + [b.id for b in self.rigid_bodies]
        body_dofs = [2] * len(self.points) + [3] * len(self.rigid_bodies)

        seg_start = 0.0
        last_t = None
        while seg_start < duration:
            seg_end = min(seg_start + seg_duration, duration)
            mask = (t_full >= seg_start) & (t_full <= seg_end)
            t_seg = t_full[mask]

            result = solve_ivp(
                integrator.rhs, [seg_start, seg_end], state,
                t_eval=t_seg, method="Radau", atol=1e-10, rtol=1e-10,
            )

            if not result.success:
                on_chunk({"error": result.message, "complete": True})
                return

            state = result.y[:, -1]
            # Drop the leading point if it duplicates the previous segment's
            # end (segment boundaries are included in both t_seg windows).
            if last_t is not None:
                keep = np.where(result.t > last_t + 1e-12)[0]
                if len(keep) == 0:
                    continue
                keep0 = keep[0]
            else:
                keep0 = 0
            chunk = {
                "t": result.t[keep0:].tolist(),
                "q": result.y[:nq, keep0:].T.tolist(),
                "qd": result.y[nq:2*nq, keep0:].T.tolist() if result.y.shape[0] >= 2*nq else None,
                "node_order": node_order,
                "complete": seg_end >= duration,
            }
            if seg_start == 0.0:
                chunk["body_dofs"] = body_dofs
            last_t = float(result.t[-1])
            on_chunk(chunk)
            seg_start = seg_end

    # ─── Event-Driven Methods ──────────────────────────────────────────

    def _event_func_from_state(self, body_id_a, body_id_b):
        """Return a closure for solve_ivp to use as event(t, state) for two point-masses."""
        body_a = self._find_body(body_id_a)
        body_b = self._find_body(body_id_b)
        if body_a is None or body_b is None:
            return None

        idx_a = self._body_dof_index(body_a)
        idx_b = self._body_dof_index(body_b)
        r_sum = getattr(body_a, "radius", 0.0) + getattr(body_b, "radius", 0.0)
        r_sum_sq = r_sum * r_sum

        def event(t, state):
            ax = state[idx_a]
            ay = state[idx_a + 1]
            bx = state[idx_b]
            by = state[idx_b + 1]
            dx = ax - bx
            dy = ay - by
            return dx * dx + dy * dy - r_sum_sq
        event.terminal = True
        event.direction = 0
        return event

    def _body_shape_segments(self, body, state):
        """World-frame segments of a RigidBody: rod -> 1 segment, rect -> 4."""
        if not (hasattr(body, "theta_sym") and body.theta_sym is not None):
            return []
        idx = self._body_dof_index(body)
        cx, cy, th = state[idx], state[idx + 1], state[idx + 2]
        L = float(body.params.get("length", 2.0))
        W = float(body.params.get("width", 0.5))
        half_l, half_w = L / 2.0, W / 2.0
        ct, st = np.cos(th), np.sin(th)
        if body.params.get("shape", "rect") == "rod":
            return [((cx - half_l * ct, cy - half_l * st),
                     (cx + half_l * ct, cy + half_l * st))]
        corners = [(-half_l, -half_w), (half_l, -half_w),
                   (half_l, half_w), (-half_l, half_w)]
        pts = [(cx + u * ct - v * st, cy + u * st + v * ct) for u, v in corners]
        return [(pts[i], pts[(i + 1) % 4]) for i in range(4)]

    def _shape_vs_shape_distance(self, bi, bj, state):
        """Min distance between two rigid bodies' shapes plus closest points."""
        segs_a = self._body_shape_segments(bi, state)
        segs_b = self._body_shape_segments(bj, state)
        best = None
        for a1, a2 in segs_a:
            for b1, b2 in segs_b:
                pa, pb, d = _segment_closest_points(a1, a2, b1, b2)
                if best is None or d < best[0]:
                    best = (d, pa, pb)
        return best if best is not None else (np.inf, None, None)

    def _shape_vs_anchor_distance(self, body, ax, ay, state):
        """Distance from a body's shape to an anchor point, plus the closest
        point on the shape."""
        segs = self._body_shape_segments(body, state)
        best = None
        for s1, s2 in segs:
            p, d = _point_segment_closest(ax, ay, s1[0], s1[1], s2[0], s2[1])
            if best is None or d < best[0]:
                best = (d, p)
        return best if best is not None else (np.inf, None)

    def _event_shape_vs_shape(self, idx_a, idx_b, body_a, body_b, offset):
        """Collision event between two rigid bodies (rod/rect) via their
        segments.  Covers rod-rod, rod-rect and rect-rect; `offset` is the
        capsule half-width of the rod(s)."""
        def event(t, state):
            d, _, _ = self._shape_vs_shape_distance(body_a, body_b, state)
            return d - offset
        event.terminal = True
        event.direction = 0
        return event

    def _event_point_vs_rod(self, mp_idx, rb_idx, L, contact_radius):
        """Point-to-line-segment collision event.

        Returns signed separation from the rod capsule.  Event functions must
        cross zero; a squared distance never becomes negative on penetration.
        """
        half = L / 2.0
        def event(t, state):
            px, py = state[mp_idx], state[mp_idx + 1]
            cx, cy = state[rb_idx], state[rb_idx + 1]
            theta = state[rb_idx + 2]
            ct, st = np.cos(theta), np.sin(theta)
            # Segment endpoints in world
            ax = cx - half * ct
            ay = cy - half * st
            bx = cx + half * ct
            by = cy + half * st
            # Project point onto segment line
            abx = bx - ax
            aby = by - ay
            t_param = ((px - ax) * abx + (py - ay) * aby) / (abx*abx + aby*aby + 1e-30)
            t_param = max(0.0, min(1.0, t_param))
            cx2 = ax + t_param * abx
            cy2 = ay + t_param * aby
            dx = px - cx2
            dy = py - cy2
            return np.sqrt(dx * dx + dy * dy) - contact_radius
        event.terminal = True
        event.direction = 0
        return event

    def _event_point_vs_rect(self, mp_idx, rb_idx, L, W, point_radius):
        """Point-to-rectangle collision event.

        Returns signed separation between the point circle and the rectangle.
        """
        half_l = L / 2.0
        half_w = W / 2.0
        def event(t, state):
            px, py = state[mp_idx], state[mp_idx + 1]
            cx, cy = state[rb_idx], state[rb_idx + 1]
            theta = state[rb_idx + 2]
            ct, st = np.cos(theta), np.sin(theta)
            # Transform point to body frame
            dx = px - cx
            dy = py - cy
            local_x = dx * ct + dy * st
            local_y = -dx * st + dy * ct
            # Distance to nearest point on rect
            nearest_x = max(-half_l, min(half_l, local_x))
            nearest_y = max(-half_w, min(half_w, local_y))
            rx = local_x - nearest_x
            ry = local_y - nearest_y
            return np.sqrt(rx * rx + ry * ry) - point_radius
        event.terminal = True
        event.direction = 0
        return event

    def _find_body(self, body_id):
        for p in self.points:
            if p.id == body_id:
                return p
        for b in self.rigid_bodies:
            if b.id == body_id:
                return b
        return None

    def _body_dof_index(self, body):
        if hasattr(body, "idx") and body.idx is not None:
            if hasattr(body, "theta_sym") and body.theta_sym is not None:
                return 2 * len(self.points) + 3 * body.idx
            else:
                return 2 * body.idx
        return 0

    def _build_collision_events(self):
        """Build event dicts for all pairs of bodies with radius > 0.

        Includes dynamic bodies (MassPoint, RigidBody) and static anchors
        that have radius > 0.  Every (point/rod/rect) x (point/rod/rect)
        combination gets a precise geometry event; a rect-vs-rect or
        rod-vs-rect pair no longer falls back to (or silently loses) the
        circle approximation.
        """
        restitution = float(self.topology.get("system_env", {}).get("restitution", 1.0))
        all_bodies = self.points + self.rigid_bodies
        bodies_with_radius = [b for b in all_bodies
                              if getattr(b, "radius", 0.0) > 0 and b.id is not None]
        # Precise point/rod/rect geometry events cover EVERY dynamic pair
        # (matching the pre-existing behavior: rigid bodies are colliders
        # even when they carry no radius param).  Only circle-circle
        # (point-point) and anchor events require radius > 0.
        bodies_all = [b for b in all_bodies if b.id is not None]

        # Collect anchors with radius from topology
        anchor_radius = {}
        for n in self.topology.get("nodes", []):
            if n["type"] == "Anchor":
                r = float(n.get("params", {}).get("radius", 0.0))
                if r > 0:
                    anchor_radius[n["id"]] = r

        def _is_point(body):
            return not (hasattr(body, "theta_sym") and body.theta_sym is not None)

        events = []
        resting = self._resting_pairs
        for i in range(len(bodies_all)):
            bi = bodies_all[i]
            is_point_i = _is_point(bi)

            for j in range(i + 1, len(bodies_all)):
                bj = bodies_all[j]
                if frozenset([bi.id, bj.id]) in resting:
                    continue
                is_point_j = _is_point(bj)

                if is_point_i and is_point_j:
                    if not (getattr(bi, "radius", 0.0) > 0 and
                            getattr(bj, "radius", 0.0) > 0):
                        continue
                    # Normalize to (point, rigid) when exactly one is a point,
                    # so the precise point-vs-shape events and contact math can
                    # assume body_i is the point and body_j the rigid body.
                    bi_n, bj_n = bi, bj
                elif is_point_j and not is_point_i:
                    bi_n, bj_n = bj, bi
                else:
                    bi_n, bj_n = bi, bj

                idx_i = self._body_dof_index(bi_n)
                idx_j = self._body_dof_index(bj_n)

                if is_point_i and is_point_j:
                    event_func = self._event_func_from_state(bi_n.id, bj_n.id)
                    geometry = None
                elif is_point_i or is_point_j:
                    shape_j = bj_n.params.get("shape", "rect")
                    L_b = float(bj_n.params.get("length", 2.0))
                    W_b = float(bj_n.params.get("width", 0.5))
                    if shape_j == "rod":
                        contact_radius = bi_n.radius + W_b / 2.0
                        event_func = self._event_point_vs_rod(idx_i, idx_j, L_b, contact_radius)
                        geometry = "point_rod"
                    else:
                        event_func = self._event_point_vs_rect(idx_i, idx_j, L_b, W_b, bi_n.radius)
                        geometry = "point_rect"
                else:
                    # Rigid-vs-rigid: rod/rect combinations via segments.
                    shape_i = bi_n.params.get("shape", "rect")
                    shape_j = bj_n.params.get("shape", "rect")
                    W_a = float(bi_n.params.get("width", 0.5))
                    W_b = float(bj_n.params.get("width", 0.5))
                    if shape_i == "rod" and shape_j == "rod":
                        offset = (W_a + W_b) / 2.0
                    elif shape_i == "rod":
                        offset = W_a / 2.0
                    elif shape_j == "rod":
                        offset = W_b / 2.0
                    else:
                        # Rect-vs-rect: the raw distance is >= 0 and never
                        # crosses zero (it just touches), which no terminal
                        # event would detect.  A tiny epsilon makes the event
                        # fire epsilon before contact.
                        offset = 1e-9
                    event_func = self._event_shape_vs_shape(idx_i, idx_j, bi_n, bj_n, offset)
                    geometry = "shape_shape"

                events.append({
                    "name": f"collision_{bi_n.id}_{bj_n.id}",
                    "func": event_func,
                    "terminal": True,
                    "direction": 0,
                    "type": "collision",
                    "body_i": bi_n,
                    "body_j": bj_n,
                    "restitution": restitution,
                    "geometry": geometry,
                })

            # Dynamic body vs anchor
            for anchor_id, anchor_r in anchor_radius.items():
                if frozenset([bi.id, anchor_id]) in resting:
                    continue
                anchor_node = None
                for n in self.topology.get("nodes", []):
                    if n["id"] == anchor_id:
                        anchor_node = n
                        break
                if anchor_node is None:
                    continue
                ax = float(anchor_node.get("init_pos", [0, 0])[0])
                ay = float(anchor_node.get("init_pos", [0, 0])[1])
                idx_i = self._body_dof_index(bi)

                if is_point_i:
                    event_func = self._make_anchor_point_event(idx_i, ax, ay, bi.radius, anchor_r)
                    geometry = None
                else:
                    # Rod/rect bodies collide with the anchor by their actual
                    # shape, not just the COM circle, so rod ends and rect
                    # corners cannot pass through walls/floor circles.
                    shape = bi.params.get("shape", "rect")
                    half_w = float(bi.params.get("width", 0.5)) / 2.0
                    offset = anchor_r + (half_w if shape == "rod" else 0.0)
                    event_func = self._make_anchor_shape_event(idx_i, ax, ay, bi, offset)
                    geometry = "anchor_rod" if shape == "rod" else "anchor_rect"

                events.append({
                    "name": f"collision_{bi.id}_{anchor_id}",
                    "func": event_func,
                    "terminal": True,
                    "direction": 0,
                    "type": "collision",
                    "body_i": bi,
                    "body_j": None,
                    "_anchor_id": anchor_id,
                    "_anchor_x": ax,
                    "_anchor_y": ay,
                    "_anchor_r": anchor_r,
                    "restitution": restitution,
                    "geometry": geometry,
                })

        return events

    def _make_anchor_point_event(self, idx_i, ax, ay, bi_r, anchor_r):
        """Circle-vs-circle event for a point body against a static anchor."""
        r_sum = bi_r + anchor_r
        r_sum_sq = r_sum * r_sum

        def event(t, state):
            dx = state[idx_i] - ax
            dy = state[idx_i + 1] - ay
            return dx * dx + dy * dy - r_sum_sq
        event.terminal = True
        event.direction = 0
        return event

    def _make_anchor_shape_event(self, idx_i, ax, ay, body, offset):
        """Shape-vs-circle event: fires when the body's rod/rect surface
        reaches the anchor circle."""
        def event(t, state):
            d, _ = self._shape_vs_anchor_distance(body, ax, ay, state)
            return d - offset
        event.terminal = True
        event.direction = 0
        return event

    def _contact_data(self, event_def, state):
        """Contact info for an event: (normal, pa, pb, dist).

        normal points from body_j to body_i; pa is the contact point on
        body_i, pb the contact point on body_j.  For point-vs-shape the
        normal comes from the closest point on the shape (center-to-center
        would be wrong for off-center hits); for shape-vs-shape it comes
        from the closest-point pair of the segment features.
        """
        bi = event_def["body_i"]
        bj = event_def["body_j"]
        geometry = event_def.get("geometry")
        idx_i = self._body_dof_index(bi)
        xi, yi = state[idx_i], state[idx_i + 1]

        if geometry in ("point_rod", "point_rect"):
            rigid = bj
            ridx = self._body_dof_index(rigid)
            cx, cy, theta = state[ridx], state[ridx + 1], state[ridx + 2]
            ct, st = np.cos(theta), np.sin(theta)
            local_x = (xi - cx) * ct + (yi - cy) * st
            local_y = -(xi - cx) * st + (yi - cy) * ct
            if geometry == "point_rod":
                half_length = float(rigid.params.get("length", 2.0)) / 2.0
                nearest_x = max(-half_length, min(half_length, local_x))
                nearest_y = 0.0
            else:
                half_length = float(rigid.params.get("length", 2.0)) / 2.0
                half_width = float(rigid.params.get("width", 0.5)) / 2.0
                nearest_x = max(-half_length, min(half_length, local_x))
                nearest_y = max(-half_width, min(half_width, local_y))
            pb = (cx + nearest_x * ct - nearest_y * st,
                  cy + nearest_x * st + nearest_y * ct)
            pa = (xi, yi)
            dx = pa[0] - pb[0]
            dy = pa[1] - pb[1]
            dist = np.hypot(dx, dy)
            if dist < 1e-12:
                return None
            return (dx / dist, dy / dist), pa, pb, dist

        if geometry == "shape_shape":
            d, pa, pb = self._shape_vs_shape_distance(bi, bj, state)
            if d >= np.inf:
                return None
            dx = pa[0] - pb[0]
            dy = pa[1] - pb[1]
            dist = np.hypot(dx, dy)
            if dist < 1e-12:
                # Degenerate (deep overlap): fall back to center-to-center.
                idx_j = self._body_dof_index(bj)
                xj, yj = state[idx_j], state[idx_j + 1]
                dx, dy = xi - xj, yi - yj
                dist = np.hypot(dx, dy)
                if dist < 1e-12:
                    return None
                pa, pb = (xi, yi), (xj, yj)
            return (dx / dist, dy / dist), pa, pb, dist

        if bj is None:
            # Anchor collision: normal from the anchor center toward the
            # body's contact feature.
            ax = event_def.get("_anchor_x", xi)
            ay = event_def.get("_anchor_y", yi)
            if geometry in ("anchor_rod", "anchor_rect"):
                _, pa = self._shape_vs_anchor_distance(bi, ax, ay, state)
            else:
                pa = (xi, yi)
            dx = pa[0] - ax
            dy = pa[1] - ay
            dist = np.hypot(dx, dy)
            if dist < 1e-12:
                # Center exactly on the anchor: fall back to COM direction.
                dx, dy = xi - ax, yi - ay
                dist = np.hypot(dx, dy)
                if dist < 1e-12:
                    return None
                pa = (xi, yi)
            return (dx / dist, dy / dist), pa, (ax, ay), dist

        # Point-point (circle event): normal from centers, contact at centers.
        idx_j = self._body_dof_index(bj)
        xj, yj = state[idx_j], state[idx_j + 1]
        dx = xi - xj
        dy = yi - yj
        dist = np.hypot(dx, dy)
        if dist < 1e-12:
            return None
        return (dx / dist, dy / dist), (xi, yi), (xj, yj), dist

    def _anchor_pos_func(self, body):
        """Return a callable t -> (x, y) for a static or moving anchor.

        Anchors have no state slots, so their position must be evaluated
        directly instead of read from the solver state.
        """
        xs, ys = body.x_sym, body.y_sym
        xs_has_t = hasattr(xs, "has") and xs.has(self.sm.t)
        ys_has_t = hasattr(ys, "has") and ys.has(self.sm.t)
        xf = sp.lambdify(self.sm.t, xs, modules="numpy") if xs_has_t else None
        yf = sp.lambdify(self.sm.t, ys, modules="numpy") if ys_has_t else None

        def pos(t):
            return (float(xf(t)) if xf else float(xs),
                    float(yf(t)) if yf else float(ys))
        return pos

    def _node_pivot_from_state(self, node, pivot, state, nq):
        """(px, py, vx, vy) of a node's rope attachment point from solver state.

        Rigid bodies use the local pivot offset (u, v) and include the
        rotational contribution w x r to the endpoint velocity; mass points
        use the COM directly.
        """
        idx = self._body_dof_index(node)
        x, y = state[idx], state[idx + 1]
        vx, vy = state[nq + idx], state[nq + idx + 1]
        if hasattr(node, "theta_sym") and node.theta_sym is not None:
            th = state[idx + 2]
            w = state[nq + idx + 2]
            u, v = pivot if pivot is not None else (0.0, 0.0)
            ct, st = np.cos(th), np.sin(th)
            px = x + u * ct - v * st
            py = y + u * st + v * ct
            rx, ry = px - x, py - y
            vx += -w * ry
            vy += w * rx
            return px, py, vx, vy
        return x, y, vx, vy

    def _build_soft_rope_events(self):
        events = []
        for e in self.edges:
            if e.type != "SoftRope":
                continue
            a = e.from_node
            b = e.to_node
            if a is None or b is None:
                continue

            idx_a = self._body_dof_index(a) if _is_dynamic(a) else None
            idx_b = self._body_dof_index(b) if _is_dynamic(b) else None
            pos_a = None if idx_a is not None else self._anchor_pos_func(a)
            pos_b = None if idx_b is not None else self._anchor_pos_func(b)
            pivot_a = getattr(e, "from_pivot", None)
            pivot_b = getattr(e, "to_pivot", None)
            length = float(e.length)

            def make_event(edge, direction, ia, ib, pa, pb, pva, pvb, rope_len):
                def event(t, state):
                    if ia is not None:
                        ax, ay = self._node_pivot_from_state(edge.from_node, pva, state, 0)[:2]
                    else:
                        ax, ay = pa(t)
                    if ib is not None:
                        bx, by = self._node_pivot_from_state(edge.to_node, pvb, state, 0)[:2]
                    else:
                        bx, by = pb(t)
                    return np.sqrt((ax - bx) ** 2 + (ay - by) ** 2) - rope_len
                event.terminal = True
                event.direction = direction
                return event

            # A slack rope tightens when stretched past its length.  A rope
            # that is already tight has no tighten event (it releases via
            # constraint-multiplier sign instead of a distance crossing).
            if not getattr(e, "_tight", False):
                events.append({
                    "name": f"tighten_{e.id}",
                    "func": make_event(e, +1, idx_a, idx_b, pos_a, pos_b,
                                       pivot_a, pivot_b, length),
                    "terminal": True,
                    "direction": +1,
                    "type": "tighten",
                    "edge": e,
                })
        return events

    def run_events(self, on_chunk):
        """Event-driven simulation with interrupt-restart loop.

        Uses dual-mode: if no events are needed, falls through to run_stream.
        Otherwise uses batch-mode interrupt loop.
        """
        self._step1_instantiate()
        self._step2_project()
        self._step3_energy()
        self._step4_constraints()

        self._resting_pairs = set()
        self._resting_rods = []
        # Ropes that are taut from the start (or rods added before this
        # point) must have their multipliers monitored immediately, or a
        # pushed rope could never slacken.
        self._refresh_resting_rods()

        # Check if any events are needed
        collision_events = self._build_collision_events()
        soft_rope_events = self._build_soft_rope_events()
        all_event_defs = collision_events + soft_rope_events

        if not all_event_defs and not self._resting_rods:
            self.run_stream(on_chunk, _skip_init=True)
            return

        L = self.T - self.V
        q = self.sm.q
        qd = self.sm.qd
        env = self.topology["system_env"]

        duration = float(env.get("duration", 10.0))
        max_mutations = int(env.get("max_mutations", 100))
        t_current = 0.0
        state = np.concatenate([self.q0_projected, self.qd0])
        mutation_count = 0
        nq = self.sm.nq
        node_order = [p.id for p in self.points] + [b.id for b in self.rigid_bodies]
        body_dofs = [2] * len(self.points) + [3] * len(self.rigid_bodies)

        ext_forces = self._build_external_forces()

        last_t = None

        while t_current < duration and mutation_count < max_mutations:
            integrator = NumericalIntegrator(L, q, qd, self.holonomic, self.sm, external_forces=ext_forces)

            # Release resting contacts that are in tension (their constraint
            # would pull bodies together instead of pushing them apart) and
            # ropes that are being pushed (a rope only pulls).
            if self._resting_rods:
                try:
                    lams = integrator.multipliers(t_current, state)
                except Exception:
                    lams = None
                if lams is not None:
                    released = False
                    for rod, lam_idx, kind in list(self._resting_rods):
                        if lam_idx >= len(lams):
                            continue
                        if kind == "contact" and lams[lam_idx] < -self.TENSION_EPS:
                            if rod in self.edges:
                                self.edges.remove(rod)
                            self._resting_pairs.discard(frozenset([rod.from_id, rod.to_id]))
                            released = True
                        elif kind == "rope" and lams[lam_idx] > self.TENSION_EPS:
                            rod._tight = False
                            released = True
                    if released:
                        self._refresh_resting_rods()
                        self._step3_energy()
                        self._step4_constraints()
                        L = self.T - self.V
                        nq = self.sm.nq
                        collision_events = self._build_collision_events()
                        soft_rope_events = self._build_soft_rope_events()
                        all_event_defs = collision_events + soft_rope_events
                        continue

            event_funcs = [ed["func"] for ed in all_event_defs]

            result = integrator.integrate_events(
                state[:nq], state[nq:], [t_current, duration],
                events=event_funcs,
            )

            any_triggered = (hasattr(result, "t_events")
                             and result.t_events is not None
                             and any(len(te) > 0 for te in result.t_events))

            if any_triggered:
                # Emit trajectory BEFORE the event (t=0 through t_event),
                # dropping any leading point that duplicates the previous
                # chunk's tail (restart from the event time).
                keep0 = 0
                if last_t is not None:
                    keep = np.where(result.t > last_t + 1e-12)[0]
                    keep0 = keep[0] if len(keep) else len(result.t)
                on_chunk({
                    "t": result.t[keep0:].tolist(),
                    "q": result.y[:nq, keep0:].T.tolist(),
                    "qd": result.y[nq:2*nq, keep0:].T.tolist() if result.y.shape[0] >= 2*nq else None,
                    "node_order": node_order,
                    "body_dofs": body_dofs,
                })
                last_t = float(result.t[-1])

                t_event = None
                first_idx = -1
                for ei, te_list in enumerate(result.t_events):
                    if len(te_list) > 0:
                        te = te_list[0]
                        if t_event is None or te < t_event:
                            t_event = te
                            first_idx = ei

                state_event = result.y_events[first_idx][0]
                ed = all_event_defs[first_idx]
                etype = ed["type"]

                new_state = state_event.copy()
                topology_change = False

                if etype == "collision":
                    bi = ed["body_i"]
                    bj = ed["body_j"]
                    idx_i = self._body_dof_index(bi)

                    nq_snap = nq
                    xi = new_state[idx_i]
                    yi = new_state[idx_i + 1]

                    if bj is None:
                        # Anchor collision: infinite mass.
                        ax = ed.get("_anchor_x", xi)
                        ay = ed.get("_anchor_y", yi)
                        cd = self._contact_data(ed, new_state)
                        if cd is None:
                            state = new_state
                        else:
                            (nx, ny), pa, _pb, _cd = cd
                            # Contact point on the body; r is its lever arm
                            # about the body COM.  For a rod/rect the contact
                            # is at the shape surface, not the COM circle.
                            rax, ray = pa[0] - xi, pa[1] - yi

                            vxi = new_state[nq_snap + idx_i]
                            vyi = new_state[nq_snap + idx_i + 1]
                            vcx, vcy = vxi, vyi
                            if hasattr(bi, "theta_sym") and bi.theta_sym is not None:
                                wi = new_state[nq_snap + idx_i + 2]
                                vcx += -wi * ray
                                vcy += wi * rax

                            # Bounce regardless of approach direction: a body
                            # escaping a large anchor (e.g. a floor circle) must
                            # be reflected too, otherwise it falls through.
                            e_rest = float(ed.get("restitution", 1.0))
                            vn = vcx * nx + vcy * ny
                            dvn = -(1.0 + e_rest) * vn

                            meff_inv = 1.0 / bi.m
                            if hasattr(bi, "theta_sym") and bi.theta_sym is not None:
                                rn = rax * ny - ray * nx
                                if bi.I > 0:
                                    meff_inv += rn * rn / bi.I
                            impulse = dvn / meff_inv

                            new_state[nq_snap + idx_i] += impulse / bi.m * nx
                            new_state[nq_snap + idx_i + 1] += impulse / bi.m * ny
                            if hasattr(bi, "theta_sym") and bi.theta_sym is not None:
                                torque = rax * (impulse * ny) - ray * (impulse * nx)
                                if bi.I > 0:
                                    new_state[nq_snap + idx_i + 2] += torque / bi.I

                            vn_post = vn + dvn
                            if abs(vn_post) < self.REST_EPS:
                                # Resting contact with the anchor: convert to
                                # a bilateral distance constraint (COM to
                                # anchor center) so gravity does not re-trigger
                                # events forever.
                                anchor_id = ed.get("_anchor_id")
                                d_com = np.hypot(xi - ax, yi - ay)
                                rod = IdealRod(f"rest_{bi.id}_{anchor_id}", bi.id, anchor_id,
                                               {"length": d_com})
                                rod.from_node = bi
                                rod.to_node = self.node_map.get(anchor_id)
                                rod._contact_rod = True
                                self.edges.append(rod)
                                self._resting_pairs.add(frozenset([bi.id, anchor_id]))
                                topology_change = True
                            else:
                                # Positional separation: nudge to the side the
                                # body is heading after the bounce (outward for
                                # an approach, inward for an escape), so the
                                # event does not immediately re-cross.
                                if ed.get("geometry"):
                                    sep = 1e-10
                                else:
                                    dist = np.hypot(xi - ax, yi - ay)
                                    sep = max(0.0, (bi.radius + ed.get("_anchor_r", 0.0)) - dist) * 0.5 + 1e-10
                                sep_dir = -1.0 if vn > 0 else 1.0
                                new_state[idx_i] += sep_dir * sep * nx
                                new_state[idx_i + 1] += sep_dir * sep * ny
                    else:
                        idx_j = self._body_dof_index(bj)
                        xj = new_state[idx_j]
                        yj = new_state[idx_j + 1]

                        cd = self._contact_data(ed, new_state)
                        if cd is None:
                            state = new_state
                        else:
                            (nx, ny), pa, pb, _cd = cd
                            rax, ray = pa[0] - xi, pa[1] - yi
                            rbx, rby = pb[0] - xj, pb[1] - yj

                            vxi = new_state[nq_snap + idx_i]
                            vyi = new_state[nq_snap + idx_i + 1]
                            vxj = new_state[nq_snap + idx_j]
                            vyj = new_state[nq_snap + idx_j + 1]

                            # Contact-point velocities include rotation, so an
                            # off-center hit spins both bodies correctly.
                            vai, vaj = vxi, vyi
                            vbi, vbj = vxj, vyj
                            if hasattr(bi, "theta_sym") and bi.theta_sym is not None:
                                wi = new_state[nq_snap + idx_i + 2]
                                vai += -wi * ray
                                vaj += wi * rax
                            if hasattr(bj, "theta_sym") and bj.theta_sym is not None:
                                wj = new_state[nq_snap + idx_j + 2]
                                vbi += -wj * rby
                                vbj += wj * rbx

                            vrel = nx * (vai - vbi) + ny * (vaj - vbj)
                            if vrel < 0:
                                mi = bi.m
                                mj = bj.m
                                e_restitution = float(ed.get("restitution", 1.0))
                                # Effective mass includes rotational inertia:
                                # 1/meff = 1/mi + 1/mj + (r_i x n)^2/Ii + (r_j x n)^2/Ij
                                meff_inv = 1.0 / mi + 1.0 / mj
                                if hasattr(bi, "theta_sym") and bi.theta_sym is not None:
                                    rn = rax * ny - ray * nx
                                    if bi.I > 0:
                                        meff_inv += rn * rn / bi.I
                                if hasattr(bj, "theta_sym") and bj.theta_sym is not None:
                                    rn = rbx * ny - rby * nx
                                    if bj.I > 0:
                                        meff_inv += rn * rn / bj.I
                                impulse = -(1.0 + e_restitution) * vrel / meff_inv

                                new_state[nq_snap + idx_i] += impulse / mi * nx
                                new_state[nq_snap + idx_i + 1] += impulse / mi * ny
                                new_state[nq_snap + idx_j] -= impulse / mj * nx
                                new_state[nq_snap + idx_j + 1] -= impulse / mj * ny

                                # Torque = r x J at the true contact point.
                                if hasattr(bi, "theta_sym") and bi.theta_sym is not None:
                                    torque = rax * (impulse * ny) - ray * (impulse * nx)
                                    if bi.I > 0:
                                        new_state[nq_snap + idx_i + 2] += torque / bi.I
                                if hasattr(bj, "theta_sym") and bj.theta_sym is not None:
                                    torque = rbx * (-impulse * ny) - rby * (-impulse * nx)
                                    if bj.I > 0:
                                        new_state[nq_snap + idx_j + 2] += torque / bj.I

                                vrel_post = vrel + impulse * meff_inv
                                if abs(vrel_post) < self.REST_EPS:
                                    # Resting contact: convert to a bilateral
                                    # distance constraint instead of bouncing.
                                    d_centers = np.hypot(xi - xj, yi - yj)
                                    rod = IdealRod(f"rest_{bi.id}_{bj.id}", bi.id, bj.id,
                                                   {"length": d_centers})
                                    rod.from_node = bi
                                    rod.to_node = bj
                                    rod._contact_rod = True
                                    self.edges.append(rod)
                                    self._resting_pairs.add(frozenset([bi.id, bj.id]))
                                    topology_change = True
                                else:
                                    # Event geometry already stops at contact.
                                    # A tiny nudge prevents re-triggering on
                                    # restart.
                                    if ed.get("geometry"):
                                        sep = 1e-10
                                    else:
                                        dist = np.hypot(xi - xj, yi - yj)
                                        sep = max(0.0, (getattr(bi, "radius", 0.0) + getattr(bj, "radius", 0.0)) - dist) * 0.5 + 1e-10
                                    new_state[idx_i] += sep * nx
                                    new_state[idx_i + 1] += sep * ny
                                    new_state[idx_j] -= sep * nx
                                    new_state[idx_j + 1] -= sep * ny

                elif etype == "tighten":
                    edge = ed["edge"]
                    if not getattr(edge, "_tight", False):
                        edge._tight = True
                        edge._tightened_by_event = True
                        topology_change = True
                        # Perfectly inelastic tightening: cancel the relative
                        # velocity along the rope at the attachment points
                        # (pivot-aware for rigid bodies, so the impulse also
                        # spins a body whose rope end is off-center).
                        nq_snap = nq
                        a = edge.from_node
                        b = edge.to_node
                        a_dyn, b_dyn = _is_dynamic(a), _is_dynamic(b)
                        ia = self._body_dof_index(a) if a_dyn else None
                        ib = self._body_dof_index(b) if b_dyn else None
                        pa = None if ia is not None else self._anchor_pos_func(a)
                        pb = None if ib is not None else self._anchor_pos_func(b)
                        pva = getattr(edge, "from_pivot", None)
                        pvb = getattr(edge, "to_pivot", None)
                        t_now = float(t_event)

                        if a_dyn:
                            ax, ay, vax, vay = self._node_pivot_from_state(
                                a, pva, new_state, nq_snap)
                        else:
                            ax, ay = pa(t_now)
                            vax = vay = 0.0
                        if b_dyn:
                            bx, by, vbx, vby = self._node_pivot_from_state(
                                b, pvb, new_state, nq_snap)
                        else:
                            bx, by = pb(t_now)
                            vbx = vby = 0.0

                        dx = ax - bx
                        dy = ay - by
                        d = np.sqrt(dx * dx + dy * dy) + 1e-15
                        nx, ny = dx / d, dy / d
                        dv = nx * (vax - vbx) + ny * (vay - vby)
                        if dv > 0:
                            meff_inv = 0.0
                            rax = ray = rbx = rby = 0.0
                            if a_dyn:
                                rax, ray = ax - new_state[ia], ay - new_state[ia + 1]
                                meff_inv += 1.0 / a.m
                                if hasattr(a, "theta_sym") and a.theta_sym is not None and a.I > 0:
                                    rn = rax * ny - ray * nx
                                    meff_inv += rn * rn / a.I
                            if b_dyn:
                                rbx, rby = bx - new_state[ib], by - new_state[ib + 1]
                                meff_inv += 1.0 / b.m
                                if hasattr(b, "theta_sym") and b.theta_sym is not None and b.I > 0:
                                    rn = rbx * ny - rby * nx
                                    meff_inv += rn * rn / b.I
                            if meff_inv > 0:
                                impulse = dv / meff_inv
                                if a_dyn:
                                    new_state[nq_snap + ia] -= impulse / a.m * nx
                                    new_state[nq_snap + ia + 1] -= impulse / a.m * ny
                                    if hasattr(a, "theta_sym") and a.theta_sym is not None and a.I > 0:
                                        torque = rax * (-impulse * ny) - ray * (-impulse * nx)
                                        new_state[nq_snap + ia + 2] += torque / a.I
                                if b_dyn:
                                    new_state[nq_snap + ib] += impulse / b.m * nx
                                    new_state[nq_snap + ib + 1] += impulse / b.m * ny
                                    if hasattr(b, "theta_sym") and b.theta_sym is not None and b.I > 0:
                                        torque = rbx * (impulse * ny) - rby * (impulse * nx)
                                        new_state[nq_snap + ib + 2] += torque / b.I

                if topology_change:
                    self._step3_energy()
                    self._step4_constraints()
                    self._refresh_resting_rods()
                    L = self.T - self.V
                    nq = self.sm.nq
                    state = new_state
                    collision_events = self._build_collision_events()
                    soft_rope_events = self._build_soft_rope_events()
                    all_event_defs = collision_events + soft_rope_events
                else:
                    state = new_state

                mutation_count += 1
                on_chunk({
                    "t_event": float(t_event),
                    "event": ed["name"],
                    "type": etype,
                    "complete": False,
                })
                t_current = float(t_event)
            else:
                keep0 = 0
                if last_t is not None:
                    keep = np.where(result.t > last_t + 1e-12)[0]
                    keep0 = keep[0] if len(keep) else len(result.t)
                on_chunk({
                    "t": result.t[keep0:].tolist(),
                    "q": result.y[:nq, keep0:].T.tolist(),
                    "qd": result.y[nq:2*nq, keep0:].T.tolist() if result.y.shape[0] >= 2*nq else None,
                    "node_order": node_order,
                    "body_dofs": body_dofs,
                    "complete": True,
                })
                break

        if mutation_count >= max_mutations:
            on_chunk({"error": "Max mutations exceeded", "complete": True})

    def _constraint_count(self, e):
        """Number of holonomic constraints an edge contributes (matches
        harvest_constraints ordering)."""
        if e.type == "HingeJoint":
            return 2
        if e.type == "SoftRope":
            return 1 if getattr(e, "_tight", False) else 0
        if e.type in ("IdealRod", "SmoothRail", "FixedCoordinate",
                      "LinearRelation", "DistanceSum", "AngleConstraint"):
            return 1
        return 0

    def _refresh_resting_rods(self):
        """Recompute constraint-row indices for resting-contact rods and
        tight ropes, so their multipliers can be checked for release."""
        idx = 0
        self._resting_rods = []
        for e in self.edges:
            if getattr(e, "_contact_rod", False):
                self._resting_rods.append((e, idx, "contact"))
            elif e.type == "SoftRope" and getattr(e, "_tight", False):
                self._resting_rods.append((e, idx, "rope"))
            idx += self._constraint_count(e)


class MovingAnchor:
    """An anchor whose position is a time-varying sympy expression.

    Used for non-holonomic constraints like 'x_expr': '0.5 * 2 * t**2'.
    """
    def __init__(self, id, x_expr_str, y_expr_str, init_pos, sm):
        self.id = id
        local_vars = {"t": sm.t}
        if x_expr_str:
            self.x_sym = safe_sympify(x_expr_str, local_vars)
        else:
            self.x_sym = float(init_pos[0])
        if y_expr_str:
            self.y_sym = safe_sympify(y_expr_str, local_vars)
        else:
            self.y_sym = float(init_pos[1])
        self.vx_sym = 0.0
        self.vy_sym = 0.0


class AnchorLike:
    def __init__(self, id, init_pos):
        self.id = id
        self.x_sym = float(init_pos[0])
        self.y_sym = float(init_pos[1])
        self.vx_sym = 0.0
        self.vy_sym = 0.0
