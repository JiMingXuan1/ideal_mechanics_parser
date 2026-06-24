import sympy as sp
from sympy.physics.mechanics import dynamicsymbols


class SymbolManager:
    def __init__(self):
        self.t = sp.Symbol("t")
        self.points = []
        self.rigid_bodies = []
        self.q = []
        self.qd = []
        self.nq = 0
        self.n_points = 0
        self.n_rigid = 0
        self.coord_map = {}

    def add_point(self, point):
        idx = self.n_points
        xi = dynamicsymbols(f"x{idx}")
        yi = dynamicsymbols(f"y{idx}")
        vxi = dynamicsymbols(f"x{idx}", 1)
        vyi = dynamicsymbols(f"y{idx}", 1)

        point.idx = idx
        point.x_sym = xi
        point.y_sym = yi
        point.vx_sym = vxi
        point.vy_sym = vyi

        self.points.append(point)
        self.q.extend([xi, yi])
        self.qd.extend([vxi, vyi])
        self.nq += 2
        self.n_points += 1
        self.coord_map[point.id] = (2 * idx, 2 * idx + 1)
        return idx

    def add_rigid_body(self, body):
        idx = self.n_rigid
        base = 2 * self.n_points + 3 * idx
        xi = dynamicsymbols(f"rx{idx}")
        yi = dynamicsymbols(f"ry{idx}")
        ti = dynamicsymbols(f"rt{idx}")
        vxi = dynamicsymbols(f"rx{idx}", 1)
        vyi = dynamicsymbols(f"ry{idx}", 1)
        omi = dynamicsymbols(f"rt{idx}", 1)

        body.idx = idx
        body.x_sym = xi
        body.y_sym = yi
        body.theta_sym = ti
        body.vx_sym = vxi
        body.vy_sym = vyi
        body.omega_sym = omi

        self.rigid_bodies.append(body)
        self.q.extend([xi, yi, ti])
        self.qd.extend([vxi, vyi, omi])
        self.nq += 3
        self.n_rigid += 1
        self.coord_map[body.id] = (base, base + 1)
        return idx

    def get_q0(self, points, rigid_bodies=None):
        import numpy as np
        q0 = np.zeros(self.nq)
        for p in points:
            if p.idx is not None:
                i = 2 * p.idx
                q0[i] = float(p.init_state.get("x", 0.0))
                q0[i + 1] = float(p.init_state.get("y", 0.0))
        bodies = rigid_bodies if rigid_bodies is not None else self.rigid_bodies
        for b in bodies:
            if b.idx is not None:
                base = 2 * self.n_points + 3 * b.idx
                q0[base] = float(b.init_state.get("x", 0.0))
                q0[base + 1] = float(b.init_state.get("y", 0.0))
                q0[base + 2] = float(b.init_state.get("theta", 0.0))
        return q0

    def get_qd0(self, points, rigid_bodies=None):
        import numpy as np
        qd0 = np.zeros(self.nq)
        for p in points:
            if p.idx is not None:
                i = 2 * p.idx
                qd0[i] = float(p.init_state.get("vx", 0.0))
                qd0[i + 1] = float(p.init_state.get("vy", 0.0))
        bodies = rigid_bodies if rigid_bodies is not None else self.rigid_bodies
        for b in bodies:
            if b.idx is not None:
                base = 2 * self.n_points + 3 * b.idx
                qd0[base] = float(b.init_state.get("vx", 0.0))
                qd0[base + 1] = float(b.init_state.get("vy", 0.0))
                qd0[base + 2] = float(b.init_state.get("omega", 0.0))
        return qd0
