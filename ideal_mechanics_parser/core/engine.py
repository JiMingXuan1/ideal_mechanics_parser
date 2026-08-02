import sympy as sp
import numpy as np
from scipy.integrate import solve_ivp
from .symbols import SymbolManager
from .energy import assemble_energy
from .constraints import harvest_constraints
from .projection import project_initial_state
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
        if xs.has(sp.Symbol("t")) or ys.has(sp.Symbol("t")):
            return None, None
        return float(xs), float(ys)
    except Exception:
        return None, None


def _segment_distance(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y):
    """Shortest distance between two line segments AB and CD."""
    import numpy as np

    def dist_point_seg(px, py, sx, sy, ex, ey):
        exx, eyy = ex - sx, ey - sy
        t = ((px - sx) * exx + (py - sy) * eyy) / (exx*exx + eyy*eyy + 1e-30)
        t = max(0.0, min(1.0, t))
        cx = sx + t * exx
        cy = sy + t * eyy
        dx = px - cx
        dy = py - cy
        return np.sqrt(dx * dx + dy * dy)

    d1 = dist_point_seg(b1x, b1y, a1x, a1y, a2x, a2y)
    d2 = dist_point_seg(b2x, b2y, a1x, a1y, a2x, a2y)
    d3 = dist_point_seg(a1x, a1y, b1x, b1y, b2x, b2y)
    d4 = dist_point_seg(a2x, a2y, b1x, b1y, b2x, b2y)
    return min(d1, d2, d3, d4)


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
                if p._fx_expr or p._fy_expr:
                    self._has_external_forces = True
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
                else:
                    f_func = sp.lambdify(tuple(q), f_sym, modules="numpy")
                    J_func = sp.lambdify(tuple(q), J_sym, modules="numpy")
                    q0 = project_initial_state(q0, qd0, f_func, J_func, context=ctx)

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
            chunk = {
                "t": result.t.tolist(),
                "q": result.y[:nq].T.tolist(),
                "node_order": node_order,
                "complete": seg_end >= duration,
            }
            if seg_start == 0.0:
                chunk["body_dofs"] = body_dofs
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

    def _rigid_body_segment_endpoints(self, body):
        """Return the world-coordinate endpoints (x1,y1,x2,y2) of a RigidBody's
        collision segment (for rod shape) or bounding box (for rect shape)."""
        L = float(body.params.get("length", 2.0)) / 2.0
        W = float(body.params.get("width", 0.5)) / 2.0
        # We store theta_sym, idx etc for state access
        return L, W

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

    def _event_rod_vs_rod(self, idx_a, idx_b, L_a, L_b, contact_radius):
        """Segment-to-segment shortest-distance collision event."""
        half_a = L_a / 2.0
        half_b = L_b / 2.0
        def event(t, state):
            cax, cay = state[idx_a], state[idx_a + 1]
            ta = state[idx_a + 2]
            cbx, cby = state[idx_b], state[idx_b + 1]
            tb = state[idx_b + 2]
            ct_a, st_a = np.cos(ta), np.sin(ta)
            ct_b, st_b = np.cos(tb), np.sin(tb)
            a1x = cax - half_a * ct_a
            a1y = cay - half_a * st_a
            a2x = cax + half_a * ct_a
            a2y = cay + half_a * st_a
            b1x = cbx - half_b * ct_b
            b1y = cby - half_b * st_b
            b2x = cbx + half_b * ct_b
            b2y = cby + half_b * st_b
            # Shortest distance between two segments
            # Using geometric approach: check endpoints vs other segment, plus segment intersection
            d = _segment_distance(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y)
            return d - contact_radius
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
        that have radius > 0.
        """
        restitution = float(self.topology.get("system_env", {}).get("restitution", 1.0))
        all_bodies = self.points + self.rigid_bodies
        bodies_with_radius = [b for b in all_bodies
                              if getattr(b, "radius", 0.0) > 0 and b.id is not None]

        # Collect anchors with radius from topology
        anchor_radius = {}
        for n in self.topology.get("nodes", []):
            if n["type"] == "Anchor":
                r = float(n.get("params", {}).get("radius", 0.0))
                if r > 0:
                    anchor_radius[n["id"]] = r

        events = []
        resting = self._resting_pairs
        for i in range(len(bodies_with_radius)):
            for j in range(i + 1, len(bodies_with_radius)):
                bi = bodies_with_radius[i]
                bj = bodies_with_radius[j]
                if frozenset([bi.id, bj.id]) in resting:
                    continue
                event_func = self._event_func_from_state(bi.id, bj.id)
                if event_func is None:
                    continue
                events.append({
                    "name": f"collision_{bi.id}_{bj.id}",
                    "func": event_func,
                    "terminal": True,
                    "direction": -1,
                    "type": "collision",
                    "body_i": bi,
                    "body_j": bj,
                    "restitution": restitution,
                })

            # Dynamic body vs anchor
            for anchor_id, anchor_r in anchor_radius.items():
                bi = bodies_with_radius[i]
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
                bi_r = getattr(bi, "radius", 0.0)

                def make_anchor_event(idx_i, ax, ay, bi_r, anchor_r):
                    r_sum = bi_r + anchor_r
                    r_sum_sq = r_sum * r_sum
                    def event(t, state):
                        dx = state[idx_i] - ax
                        dy = state[idx_i + 1] - ay
                        return dx * dx + dy * dy - r_sum_sq
                    event.terminal = True
                    event.direction = 0
                    return event

                events.append({
                    "name": f"collision_{bi.id}_{anchor_id}",
                    "func": make_anchor_event(idx_i, ax, ay, bi_r, anchor_r),
                    "terminal": True,
                    "direction": -1,
                    "type": "collision",
                    "body_i": bi,
                    "body_j": None,
                    "_anchor_id": anchor_id,
                    "_anchor_x": ax,
                    "_anchor_y": ay,
                    "_anchor_r": anchor_r,
                    "restitution": restitution,
                })

        # RigidBody collisions: point-vs-rod and rod-vs-rod
        rigid_bodies_list = [b for b in self.rigid_bodies if b.id is not None]
        for bi in self.points + rigid_bodies_list:
            for bj in rigid_bodies_list:
                if bi.id == bj.id:
                    continue
                if frozenset([bi.id, bj.id]) in resting:
                    continue
                # Replace any coarse circle-vs-circle event for this pair
                # with the precise geometry event.
                already_idx = None
                for k, ed in enumerate(events):
                    if (ed.get("body_i") is bi and ed.get("body_j") is bj or
                            ed.get("body_i") is bj and ed.get("body_j") is bi):
                        already_idx = k
                        break
                if already_idx is not None:
                    events.pop(already_idx)
                shape_b = bj.params.get("shape", "rect")
                idx_i = self._body_dof_index(bi)
                idx_j = self._body_dof_index(bj)
                L_b = float(bj.params.get("length", 2.0))
                W_b = float(bj.params.get("width", 0.5))
                is_point = not (hasattr(bi, "theta_sym") and bi.theta_sym is not None)
                if is_point:
                    if shape_b == "rod":
                        contact_radius = getattr(bi, "radius", 0.0) + W_b / 2.0
                        func = self._event_point_vs_rod(idx_i, idx_j, L_b, contact_radius)
                        geometry = "point_rod"
                    else:
                        contact_radius = getattr(bi, "radius", 0.0)
                        func = self._event_point_vs_rect(idx_i, idx_j, L_b, W_b, contact_radius)
                        geometry = "point_rect"
                elif shape_b == "rod":
                    L_a = float(bi.params.get("length", 2.0))
                    W_a = float(bi.params.get("width", 0.5))
                    contact_radius = (W_a + W_b) / 2.0
                    func = self._event_rod_vs_rod(idx_i, idx_j, L_a, L_b, contact_radius)
                    geometry = "rod_rod"
                else:
                    # rect-vs-rect can be added later; skip for now
                    continue
                events.append({
                    "name": f"collision_{bi.id}_{bj.id}",
                    "func": func,
                    "terminal": True,
                    "direction": 0,
                    "type": "collision",
                    "body_i": bi,
                    "body_j": bj,
                    "restitution": restitution,
                    "geometry": geometry,
                })
        return events

    def _rigid_contact_normal(self, event_def, state):
        """Return the contact normal pointing from body_j to body_i.

        The center-to-center vector is wrong for an off-center hit on a rod or
        rectangle, so derive the normal from the closest point on the shape.
        """
        if event_def.get("geometry") not in {"point_rod", "point_rect"}:
            return None

        point = event_def["body_i"]
        rigid = event_def["body_j"]
        point_idx = self._body_dof_index(point)
        rigid_idx = self._body_dof_index(rigid)
        px, py = state[point_idx], state[point_idx + 1]
        cx, cy, theta = state[rigid_idx], state[rigid_idx + 1], state[rigid_idx + 2]
        ct, st = np.cos(theta), np.sin(theta)
        local_x = (px - cx) * ct + (py - cy) * st
        local_y = -(px - cx) * st + (py - cy) * ct

        if event_def["geometry"] == "point_rod":
            half_length = float(rigid.params.get("length", 2.0)) / 2.0
            nearest_x = max(-half_length, min(half_length, local_x))
            nearest_y = 0.0
        else:
            half_length = float(rigid.params.get("length", 2.0)) / 2.0
            half_width = float(rigid.params.get("width", 0.5)) / 2.0
            nearest_x = max(-half_length, min(half_length, local_x))
            nearest_y = max(-half_width, min(half_width, local_y))

        dx_local, dy_local = local_x - nearest_x, local_y - nearest_y
        distance = np.hypot(dx_local, dy_local)
        if distance < 1e-12:
            return None
        return ((dx_local * ct - dy_local * st) / distance,
                (dx_local * st + dy_local * ct) / distance)

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
            length = float(e.length)

            def make_event(edge, direction, ia, ib, pa, pb, rope_len):
                def event(t, state):
                    if ia is not None:
                        ax, ay = float(state[ia]), float(state[ia + 1])
                    else:
                        ax, ay = pa(t)
                    if ib is not None:
                        bx, by = float(state[ib]), float(state[ib + 1])
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
                    "func": make_event(e, +1, idx_a, idx_b, pos_a, pos_b, length),
                    "terminal": True,
                    "direction": -1,
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

        # Check if any events are needed
        collision_events = self._build_collision_events()
        soft_rope_events = self._build_soft_rope_events()
        all_event_defs = collision_events + soft_rope_events

        if not all_event_defs:
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
                # Emit trajectory BEFORE the event (t=0 through t_event)
                on_chunk({
                    "t": result.t.tolist(),
                    "q": result.y[:nq].T.tolist(),
                    "qd": result.y[nq:2*nq].T.tolist() if result.y.shape[0] >= 2*nq else None,
                    "node_order": node_order,
                    "body_dofs": body_dofs,
                })

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
                        vxi = new_state[nq_snap + idx_i]
                        vyi = new_state[nq_snap + idx_i + 1]

                        dx = xi - (ed.get("_anchor_x", xi + 1))
                        dy = yi - (ed.get("_anchor_y", yi + 1))
                        dist = np.sqrt(dx * dx + dy * dy)
                        if dist < 1e-15:
                            dist = 1e-15
                        nx = dx / dist
                        ny = dy / dist

                        # Bounce regardless of approach direction: a body
                        # escaping a large anchor (e.g. a floor circle) must
                        # be reflected too, otherwise it falls through.
                        e_rest = float(ed.get("restitution", 1.0))
                        vn = vxi * nx + vyi * ny
                        dvn = -(1.0 + e_rest) * vn
                        new_state[nq_snap + idx_i] += dvn * nx
                        new_state[nq_snap + idx_i + 1] += dvn * ny

                        # Angular impulse for rigid bodies hitting an anchor
                        if hasattr(bi, "theta_sym") and bi.theta_sym is not None:
                            omega_idx = nq_snap + idx_i + 2
                            bi_r = float(getattr(bi, "radius", 0.0))
                            cx = xi + nx * bi_r
                            cy = yi + ny * bi_r
                            rx = cx - xi
                            ry = cy - yi
                            Jx = bi.m * dvn * nx
                            Jy = bi.m * dvn * ny
                            torque = rx * Jy - ry * Jx
                            if bi.I > 0:
                                new_state[omega_idx] += torque / bi.I

                        vn_post = vn + dvn
                        if abs(vn_post) < self.REST_EPS:
                            # Resting contact with the anchor: convert to a
                            # bilateral distance constraint so gravity does
                            # not re-trigger events forever.
                            anchor_id = ed.get("_anchor_id")
                            r_sum = getattr(bi, "radius", 0.0) + ed.get("_anchor_r", 0.0)
                            rod = IdealRod(f"rest_{bi.id}_{anchor_id}", bi.id, anchor_id,
                                           {"length": r_sum})
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
                            sep = max(0.0, (bi.radius + ed.get("_anchor_r", 0.0)) - dist) * 0.5 + 1e-10
                            sep_dir = -1.0 if vn > 0 else 1.0
                            new_state[idx_i] += sep_dir * sep * nx
                            new_state[idx_i + 1] += sep_dir * sep * ny
                    else:
                        idx_j = self._body_dof_index(bj)
                        xj = new_state[idx_j]
                        yj = new_state[idx_j + 1]
                        dx = xi - xj
                        dy = yi - yj
                        dist = np.sqrt(dx * dx + dy * dy)
                        if dist < 1e-15:
                            dist = 1e-15
                        nx = dx / dist
                        ny = dy / dist

                        contact_normal = self._rigid_contact_normal(ed, new_state)
                        if contact_normal is not None:
                            nx, ny = contact_normal

                        vxi = new_state[nq_snap + idx_i]
                        vyi = new_state[nq_snap + idx_i + 1]
                        vxj = new_state[nq_snap + idx_j]
                        vyj = new_state[nq_snap + idx_j + 1]
                        vrel = nx * (vxi - vxj) + ny * (vyi - vyj)
                        if vrel < 0:
                            mi = bi.m
                            mj = bj.m
                            e_restitution = float(ed.get("restitution", 1.0))
                            impulse = -(1.0 + e_restitution) * vrel / (1.0 / mi + 1.0 / mj)

                            new_state[nq_snap + idx_i] += impulse / mi * nx
                            new_state[nq_snap + idx_i + 1] += impulse / mi * ny
                            new_state[nq_snap + idx_j] -= impulse / mj * nx
                            new_state[nq_snap + idx_j + 1] -= impulse / mj * ny

                            # Angular impulse for RigidBody collisions
                            # Torque = r x J where r = contact_point - COM,
                            # contact point is between the two bodies.
                            for (body, idx, side) in [(bi, idx_i, -1), (bj, idx_j, 1)]:
                                if hasattr(body, "theta_sym") and body.theta_sym is not None:
                                    theta_idx = idx + 2
                                    omega_idx = nq_snap + theta_idx
                                    cx = (xi + xj) / 2
                                    cy = (yi + yj) / 2
                                    rx = cx - new_state[idx]
                                    ry = cy - new_state[idx + 1]
                                    Jx = impulse / body.m * nx * (-side)
                                    Jy = impulse / body.m * ny * (-side)
                                    torque = rx * Jy - ry * Jx
                                    I_val = body.I
                                    if I_val > 0:
                                        new_state[omega_idx] += torque / I_val

                            vrel_post = vrel + impulse * (1.0 / mi + 1.0 / mj)
                            if abs(vrel_post) < self.REST_EPS:
                                # Resting contact: convert to a bilateral
                                # distance constraint instead of bouncing.
                                rod = IdealRod(f"rest_{bi.id}_{bj.id}", bi.id, bj.id,
                                               {"length": dist})
                                rod.from_node = bi
                                rod.to_node = bj
                                rod._contact_rod = True
                                self.edges.append(rod)
                                self._resting_pairs.add(frozenset([bi.id, bj.id]))
                                topology_change = True
                            else:
                                # Event geometry already stops at contact. A
                                # tiny nudge prevents re-triggering on restart.
                                sep = 1e-10 if ed.get("geometry") else (
                                    max(0.0, (getattr(bi, "radius", 0.0) + getattr(bj, "radius", 0.0)) - dist) * 0.5 + 1e-10
                                )
                                new_state[idx_i] += sep * nx
                                new_state[idx_i + 1] += sep * ny
                                new_state[idx_j] -= sep * nx
                                new_state[idx_j + 1] -= sep * ny

                elif etype == "tighten":
                    edge = ed["edge"]
                    if getattr(edge, "_tight", False):
                        pass
                    else:
                        edge._tight = True
                        edge._tightened_by_event = True
                        topology_change = True
                        # Perfectly inelastic tightening: cancel relative
                        # velocity along the rope.
                        nq_snap = nq
                        a = edge.from_node
                        b = edge.to_node
                        a_dyn, b_dyn = _is_dynamic(a), _is_dynamic(b)
                        ia = self._body_dof_index(a) if a_dyn else None
                        ib = self._body_dof_index(b) if b_dyn else None
                        pa = None if ia is not None else self._anchor_pos_func(a)
                        pb = None if ib is not None else self._anchor_pos_func(b)
                        t_now = float(t_event)
                        if ia is not None:
                            ax, ay = new_state[ia], new_state[ia + 1]
                        else:
                            ax, ay = pa(t_now)
                        if ib is not None:
                            bx, by = new_state[ib], new_state[ib + 1]
                        else:
                            bx, by = pb(t_now)
                        dx = ax - bx
                        dy = ay - by
                        d = np.sqrt(dx * dx + dy * dy) + 1e-15
                        nx, ny = dx / d, dy / d
                        vax = new_state[nq_snap + ia] if a_dyn else 0.0
                        vay = new_state[nq_snap + ia + 1] if a_dyn else 0.0
                        vbx = new_state[nq_snap + ib] if b_dyn else 0.0
                        vby = new_state[nq_snap + ib + 1] if b_dyn else 0.0
                        dv = nx * (vax - vbx) + ny * (vay - vby)
                        if dv > 0:
                            if a_dyn and b_dyn:
                                ma, mb = a.m, b.m
                                if ma > 0 and mb > 0:
                                    impulse = dv / (1.0 / ma + 1.0 / mb)
                                    new_state[nq_snap + ia] -= impulse / ma * nx
                                    new_state[nq_snap + ia + 1] -= impulse / ma * ny
                                    new_state[nq_snap + ib] += impulse / mb * nx
                                    new_state[nq_snap + ib + 1] += impulse / mb * ny
                            elif a_dyn:
                                new_state[nq_snap + ia] -= dv * nx
                                new_state[nq_snap + ia + 1] -= dv * ny
                            elif b_dyn:
                                new_state[nq_snap + ib] += dv * nx
                                new_state[nq_snap + ib + 1] += dv * ny

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
                on_chunk({
                    "t": result.t.tolist(),
                    "q": result.y[:nq].T.tolist(),
                    "qd": result.y[nq:2*nq].T.tolist() if result.y.shape[0] >= 2*nq else None,
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
