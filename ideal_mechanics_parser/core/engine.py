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

    def run(self):
        self._step1_instantiate()
        self._step2_project()
        self._step3_energy()
        self._step4_constraints()
        result = self._step5_integrate()
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

        for e in self.edges:
            e.from_node = node_map.get(e.from_id)
            if e.to_id:
                e.to_node = node_map.get(e.to_id)

            if hasattr(e, "via_id") and e.via_id:
                e.via_node = node_map.get(e.via_id)

            if e.from_node is None:
                raise ValueError(f"Edge {e.id}: from_node '{e.from_id}' not found")

    def _step2_project(self):
        q0 = self.sm.get_q0(self.points, self.rigid_bodies)
        qd0 = self.sm.get_qd0(self.points, self.rigid_bodies)

        if self._has_constraints():
            L, _ = assemble_energy(self.points, self.edges, self.topology["system_env"],
                                   self.sm, self.rigid_bodies)
            holonomic = harvest_constraints(self.edges, self.sm)
            q = self.sm.q

            if holonomic:
                f_sym = sp.Matrix(holonomic)
                J_sym = f_sym.jacobian(q)
                has_t = any(f.has(self.sm.t) for f in holonomic)
                if has_t:
                    proj_args = tuple(q) + (self.sm.t,)
                    f_func = sp.lambdify(proj_args, f_sym, modules="numpy")
                    J_func = sp.lambdify(proj_args, J_sym, modules="numpy")
                    q0 = project_initial_state(q0, qd0, f_func, J_func, extra_args=(0.0,))
                else:
                    f_func = sp.lambdify(tuple(q), f_sym, modules="numpy")
                    J_func = sp.lambdify(tuple(q), J_sym, modules="numpy")
                    q0 = project_initial_state(q0, qd0, f_func, J_func)

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
        for p in self.points:
            fx = getattr(p, "_fx_expr", None)
            fy = getattr(p, "_fy_expr", None)
            if fx:
                expr = safe_sympify(fx, {"t": self.sm.t})
                forces.append((self._body_dof_index(p), expr))
            if fy:
                expr = safe_sympify(fy, {"t": self.sm.t})
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

    def _event_point_vs_rod(self, mp_idx, rb_idx, L):
        """Point-to-line-segment collision event.

        Returns dx^2 where dx is the shortest distance from point to rod segment.
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
            return dx * dx + dy * dy
        event.terminal = True
        event.direction = 0
        return event

    def _event_rod_vs_rod(self, idx_a, idx_b, L_a, L_b):
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
            return d * d
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
        for i in range(len(bodies_with_radius)):
            for j in range(i + 1, len(bodies_with_radius)):
                bi = bodies_with_radius[i]
                bj = bodies_with_radius[j]
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
                # Check if this pair already has a circle-vs-circle event
                already = any(ed.get("body_i") is bi and ed.get("body_j") is bj or
                             ed.get("body_i") is bj and ed.get("body_j") is bi
                             for ed in events)
                if already:
                    continue
                shape_b = bj.params.get("shape", "rect")
                idx_i = self._body_dof_index(bi)
                idx_j = self._body_dof_index(bj)
                L_b = float(bj.params.get("length", 2.0))
                is_point = not (hasattr(bi, "theta_sym") and bi.theta_sym is not None)
                if is_point and shape_b == "rod":
                    func = self._event_point_vs_rod(idx_i, idx_j, L_b)
                elif not is_point and shape_b == "rod":
                    L_a = float(bi.params.get("length", 2.0))
                    func = self._event_rod_vs_rod(idx_i, idx_j, L_a, L_b)
                else:
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
                })
        return events

    def _build_soft_rope_events(self):
        events = []
        for e in self.edges:
            if e.type != "SoftRope":
                continue
            a = e.from_node
            b = e.to_node
            if a is None or b is None:
                continue

            idx_a = self._body_dof_index(a)
            idx_b = self._body_dof_index(b)

            def make_tighten_event(idx_a, idx_b, length):
                def event(t, state):
                    dx = state[idx_a] - state[idx_b]
                    dy = state[idx_a + 1] - state[idx_b + 1]
                    return np.sqrt(dx * dx + dy * dy) - length
                event.terminal = True
                event.direction = +1
                return event

            def make_slacken_event(idx_a, idx_b, length):
                def event(t, state):
                    dx = state[idx_a] - state[idx_b]
                    dy = state[idx_a + 1] - state[idx_b + 1]
                    return np.sqrt(dx * dx + dy * dy) - length
                event.terminal = True
                event.direction = -1
                return event

            events.append({
                "name": f"tighten_{e.id}",
                "func": make_tighten_event(idx_a, idx_b, e.length),
                "terminal": True,
                "direction": -1,
                "type": "tighten",
                "edge": e,
            })
            events.append({
                "name": f"slacken_{e.id}",
                "func": make_slacken_event(idx_a, idx_b, e.length),
                "terminal": True,
                "direction": +1,
                "type": "slacken",
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
                        # Anchor collision: infinite mass, reverse velocity
                        vxi = new_state[nq_snap + idx_i]
                        vyi = new_state[nq_snap + idx_i + 1]

                        dx = xi - (ed.get("_anchor_x", xi + 1))
                        dy = yi - (ed.get("_anchor_y", yi + 1))
                        dist = np.sqrt(dx * dx + dy * dy)
                        if dist < 1e-15:
                            dist = 1e-15
                        nx = dx / dist
                        ny = dy / dist

                        vn = vxi * nx + vyi * ny
                        if vn < 0:
                            e = float(ed.get("restitution", 1.0))
                            new_state[nq_snap + idx_i] -= (1 + e) * vn * nx
                            new_state[nq_snap + idx_i + 1] -= (1 + e) * vn * ny
                            # Positional separation from anchor
                            sep = max(0.0, (bi.radius + ed.get("_anchor_r", 0.0)) - dist) * 0.5 + 1e-10
                            new_state[idx_i] += sep * nx
                            new_state[idx_i + 1] += sep * ny
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
                            # contact point is at the collision location (midpoint)
                            for (body, idx, side) in [(bi, idx_i, -1), (bj, idx_j, 1)]:
                                if hasattr(body, "theta_sym") and body.theta_sym is not None:
                                    dof = 3
                                    theta_idx = idx + 2
                                    omega_idx = nq_snap + theta_idx
                                    # Contact point is at bi's position for anchor collision,
                                    # or between the two bodies
                                    if bj is None:
                                        cx = (xi + ed.get("_anchor_x", xi)) / 2
                                        cy = (yi + ed.get("_anchor_y", yi)) / 2
                                    else:
                                        cx = (xi + xj) / 2
                                        cy = (yi + yj) / 2
                                    rx = cx - new_state[idx]
                                    ry = cy - new_state[idx + 1]
                                    # 2D torque: tau = r x J = rx * (J*ny) - ry * (J*nx)
                                    # where J_linear = J * n (the impulse vector)
                                    Jx = impulse / body.m * nx * (-side)
                                    Jy = impulse / body.m * ny * (-side)
                                    torque = rx * Jy - ry * Jx
                                    I_val = body.I
                                    if I_val > 0:
                                        new_state[omega_idx] += torque / I_val

                            # Positional separation: push apart along normal to prevent re-trigger
                            sep = max(0.0, (getattr(bi, "radius", 0.0) + getattr(bj, "radius", 0.0)) - dist) * 0.5 + 1e-10
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
                        topology_change = True
                        # Perfectly inelastic tightening: cancel relative velocity along rope
                        nq_snap = nq
                        a = edge.from_node
                        b = edge.to_node
                        ia = self._body_dof_index(a)
                        ib = self._body_dof_index(b)
                        dx = new_state[ia] - new_state[ib]
                        dy = new_state[ia + 1] - new_state[ib + 1]
                        d = np.sqrt(dx * dx + dy * dy) + 1e-15
                        nx, ny = dx / d, dy / d
                        dv = nx * (new_state[nq_snap + ia] - new_state[nq_snap + ib]) + ny * (new_state[nq_snap + ia + 1] - new_state[nq_snap + ib + 1])
                        ma, mb = a.m, b.m
                        if dv > 0 and ma > 0 and mb > 0:
                            impulse = dv / (1.0 / ma + 1.0 / mb)
                            new_state[nq_snap + ia] -= impulse / ma * nx
                            new_state[nq_snap + ia + 1] -= impulse / ma * ny
                            new_state[nq_snap + ib] += impulse / mb * nx
                            new_state[nq_snap + ib + 1] += impulse / mb * ny

                elif etype == "slacken":
                    edge = ed["edge"]
                    if hasattr(edge, "_tight") and edge._tight:
                        edge._tight = False
                        topology_change = True

                if topology_change:
                    self._apply_soft_rope_constraints()
                    self._step3_energy()
                    self._step4_constraints()
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

    def _apply_soft_rope_constraints(self):
        """Replace SoftRope edges with IdealRod or remove them based on _tight state.

        Mutates self.edges in-place without touching self.topology or re-instantiating.
        """
        new_edges = []
        for e in self.edges:
            if e.type == "SoftRope":
                if getattr(e, "_tight", False):
                    rod = IdealRod(e.id, e.from_id, e.to_id, {"length": e.length})
                    rod.from_node = e.from_node
                    rod.to_node = e.to_node
                    new_edges.append(rod)
                # slack: no constraint contributed
            else:
                new_edges.append(e)
        self.edges = new_edges


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
