import sympy as sp
import numpy as np
from scipy.integrate import solve_ivp
from .symbols import SymbolManager
from .energy import assemble_energy
from .constraints import harvest_constraints
from .projection import project_initial_state
from .numerical import NumericalIntegrator
from entities.mass_point import MassPoint
from entities.ideal_rod import IdealRod
from entities.ideal_spring import IdealSpring
from entities.smooth_rail import SmoothRail
from entities.fixed_coordinate import FixedCoordinate
from entities.linear_relation import LinearRelation
from entities.distance_sum import DistanceSum
from entities.angle_constraint import AngleConstraint


class Engine:
    def __init__(self, topology):
        self.topology = topology
        self.sm = SymbolManager()
        self.points = []
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
            if node["type"] == "MassPoint":
                p = MassPoint(node["id"], node.get("init_state"), node.get("params"))
                p.m = float(node.get("params", {}).get("m", 1.0))
                self.sm.add_point(p)
                self.points.append(p)

        _EDGE_TYPES = {
            "IdealRod": IdealRod,
            "IdealSpring": IdealSpring,
            "SmoothRail": SmoothRail,
            "FixedCoordinate": FixedCoordinate,
            "LinearRelation": LinearRelation,
            "DistanceSum": DistanceSum,
            "AngleConstraint": AngleConstraint,
        }

        for edge_data in self.topology["edges"]:
            cls = _EDGE_TYPES.get(edge_data["type"])
            if cls is None:
                continue
            e = cls(edge_data["id"], edge_data.get("from"), edge_data.get("to"), edge_data.get("params"))
            self.edges.append(e)

        all_nodes = {n["id"]: n for n in self.topology["nodes"]}
        for e in self.edges:
            for p in self.points:
                if p.id == e.from_id:
                    e.from_node = p
                if p.id == e.to_id:
                    e.to_node = p

            if e.from_node is None or (e.to_node is None and e.type not in ("FixedCoordinate", "LinearRelation")):
                for n in self.topology["nodes"]:
                    if n["type"] == "Anchor":
                        if n["id"] == e.from_id:
                            e.from_node = AnchorLike(n["id"], n.get("init_pos", [0, 0]))
                        if n["id"] == e.to_id:
                            e.to_node = AnchorLike(n["id"], n.get("init_pos", [0, 0]))

            if hasattr(e, "via_id") and e.via_id:
                for p in self.points:
                    if p.id == e.via_id:
                        e.via_node = p
                if getattr(e, "via_node", None) is None:
                    for n in self.topology["nodes"]:
                        if n["type"] == "Anchor" and n["id"] == e.via_id:
                            e.via_node = AnchorLike(n["id"], n.get("init_pos", [0, 0]))

    def _step2_project(self):
        q0 = self.sm.get_q0(self.points)
        qd0 = self.sm.get_qd0(self.points)

        if self._has_constraints():
            L, _ = assemble_energy(self.points, self.edges, self.topology["system_env"], self.sm)
            holonomic = harvest_constraints(self.edges, self.sm)
            q = self.sm.q

            if holonomic:
                f_sym = sp.Matrix(holonomic)
                J_sym = f_sym.jacobian(q)
                f_func = sp.lambdify(tuple(q), f_sym, modules="numpy")
                J_func = sp.lambdify(tuple(q), J_sym, modules="numpy")
                q0 = project_initial_state(q0, qd0, f_func, J_func)

        self.q0_projected = q0
        self.qd0 = qd0

    def _has_constraints(self):
        CONSTRAINT_TYPES = {"IdealRod", "SmoothRail", "FixedCoordinate", "LinearRelation",
                           "DistanceSum", "AngleConstraint"}
        for e in self.edges:
            if e.type in CONSTRAINT_TYPES:
                return True
        return False

    def _step3_energy(self):
        env = self.topology["system_env"]
        self.T, self.V = assemble_energy(self.points, self.edges, env, self.sm)

    def _step4_constraints(self):
        self.holonomic = harvest_constraints(self.edges, self.sm)

    def _step5_integrate(self):
        L = self.T - self.V
        q = self.sm.q
        qd = self.sm.qd
        env = self.topology["system_env"]

        integrator = NumericalIntegrator(L, q, qd, self.holonomic, self.sm)

        duration = float(env.get("duration", 10.0))
        time_step = float(env.get("time_step", 0.01))
        t_eval = np.arange(0.0, duration + time_step, time_step)

        t, q_traj, qd_traj = integrator.integrate(self.q0_projected, self.qd0, t_eval)
        return {
            "t": t.tolist(),
            "q": q_traj.tolist(),
            "qd": qd_traj.tolist(),
            "node_order": [p.id for p in self.points],
        }

    def run_stream(self, on_chunk, seg_duration=0.5):
        self._step1_instantiate()
        self._step2_project()
        self._step3_energy()
        self._step4_constraints()
        L = self.T - self.V
        q = self.sm.q
        qd = self.sm.qd
        env = self.topology["system_env"]
        integrator = NumericalIntegrator(L, q, qd, self.holonomic, self.sm)

        dt = float(env.get("time_step", 0.01))
        duration = float(env.get("duration", 10.0))
        t_full = np.arange(0.0, duration + dt, dt)
        state = np.concatenate([self.q0_projected, self.qd0])
        nq = self.sm.nq
        node_order = [p.id for p in self.points]

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
            on_chunk({
                "t": result.t.tolist(),
                "q": result.y[:nq].T.tolist(),
                "node_order": node_order,
                "complete": seg_end >= duration,
            })
            seg_start = seg_end


class AnchorLike:
    def __init__(self, id, init_pos):
        self.id = id
        self.x_sym = float(init_pos[0])
        self.y_sym = float(init_pos[1])
        self.vx_sym = 0.0
        self.vy_sym = 0.0
