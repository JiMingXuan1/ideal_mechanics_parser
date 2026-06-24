import numpy as np
import sympy as sp
from scipy.integrate import solve_ivp
from sympy.physics.mechanics import LagrangesMethod


class NumericalIntegrator:
    BAUMGARTE_ALPHA = 1.0
    BAUMGARTE_BETA = 1.0

    def __init__(self, L, q, qd, holonomic_constraints, sm):
        self.sm = sm
        self.nq = len(q)
        self.nc = len(holonomic_constraints)

        self.LM = LagrangesMethod(L, q, hol_coneqs=holonomic_constraints)
        self.LM.form_lagranges_equations()

        M_full_sym = self.LM.mass_matrix_full
        F_full_sym = self.LM.forcing_full

        t_sym = sm.t
        M_args = tuple(q)
        F_args = tuple(q + qd + [t_sym])

        self.M_full_func = sp.lambdify(M_args, M_full_sym, modules="numpy")
        self.F_full_func = sp.lambdify(F_args, F_full_sym, modules="numpy")

        if self.nc > 0:
            f_sym = sp.Matrix(holonomic_constraints)
            J_sym = f_sym.jacobian(q)
            self.J_func = sp.lambdify(tuple(q), J_sym, modules="numpy")
            self.f_func = sp.lambdify(tuple(q), f_sym, modules="numpy")
        else:
            self.J_func = None
            self.f_func = None

    def rhs(self, t, state):
        q = state[:self.nq]
        qd = state[self.nq:]

        M_full = np.asarray(self.M_full_func(*q))
        F_full = np.asarray(self.F_full_func(*(list(q) + list(qd) + [t]))).ravel()

        if self.nc > 0:
            J_val = np.asarray(self.J_func(*q))
            f_val = np.asarray(self.f_func(*q)).ravel()
            f_dot = J_val @ qd
            cons_start = 2 * self.nq
            F_full[cons_start:] = F_full[cons_start:] - 2.0 * self.BAUMGARTE_ALPHA * f_dot - self.BAUMGARTE_BETA**2 * f_val

        sol = np.linalg.lstsq(M_full, F_full, rcond=None)[0].ravel()
        qdd = sol[self.nq:2 * self.nq]
        return np.concatenate([qd, qdd])

    def integrate(self, q0, qd0, t_eval, method="Radau", atol=1e-10, rtol=1e-10):
        state0 = np.concatenate([q0, qd0])
        t_span = (t_eval[0], t_eval[-1])

        result = solve_ivp(
            self.rhs,
            t_span,
            state0,
            method=method,
            t_eval=t_eval,
            events=None,
            atol=atol,
            rtol=rtol,
        )

        if not result.success:
            raise RuntimeError(f"Integration failed: {result.message}")

        return result.t, result.y[:self.nq].T, result.y[self.nq:].T

    def integrate_events(self, q0, qd0, t_span, method="Radau", atol=1e-10, rtol=1e-10, events=None):
        """Integrate with events, no fixed t_eval.

        Returns the raw solve_ivp result (t_events, y_events accessible).
        """
        state0 = np.concatenate([q0, qd0])

        result = solve_ivp(
            self.rhs,
            t_span,
            state0,
            method=method,
            events=events,
            atol=atol,
            rtol=rtol,
            max_step=0.1,
        )

        return result
