import sympy as sp
from sympy.physics.mechanics import dynamicsymbols


class SymbolManager:
    def __init__(self):
        self.t = sp.Symbol("t")
        self.points = []
        self.q = []
        self.qd = []
        self.nq = 0
        self.coord_map = {}

    def add_point(self, point):
        idx = len(self.points)
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
        self.coord_map[point.id] = (2 * idx, 2 * idx + 1)
        return idx

    def get_q0(self, points):
        import numpy as np
        q0 = np.zeros(self.nq)
        for p in points:
            if p.idx is not None:
                i = 2 * p.idx
                q0[i] = float(p.init_state.get("x", 0.0))
                q0[i + 1] = float(p.init_state.get("y", 0.0))
        return q0

    def get_qd0(self, points):
        import numpy as np
        qd0 = np.zeros(self.nq)
        for p in points:
            if p.idx is not None:
                i = 2 * p.idx
                qd0[i] = float(p.init_state.get("vx", 0.0))
                qd0[i + 1] = float(p.init_state.get("vy", 0.0))
        return qd0
