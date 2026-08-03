import numpy as np
import sympy as sp
from scipy.integrate import solve_ivp
from sympy.physics.mechanics import LagrangesMethod


class NumericalIntegrator:
    BAUMGARTE_ALPHA = 1.0
    BAUMGARTE_BETA = 1.0

    def __init__(self, L, q, qd, holonomic_constraints, sm, external_forces=None):
        self.sm = sm
        self.nq = len(q)
        self.nc = len(holonomic_constraints)

        self.LM = LagrangesMethod(L, q, hol_coneqs=holonomic_constraints)
        self.LM.form_lagranges_equations()

        M_full_sym = self.LM.mass_matrix_full
        F_full_sym = self.LM.forcing_full

        t_sym = sm.t
        self._M_has_t = M_full_sym.has(t_sym)
        M_args = tuple(q) + (t_sym,) if self._M_has_t else tuple(q)
        F_args = tuple(q + qd + [t_sym])

        self.M_full_func = sp.lambdify(M_args, M_full_sym, modules="numpy")
        self.F_full_func = sp.lambdify(F_args, F_full_sym, modules="numpy")

        self._con_has_t = False
        self._f_t_step = 0.0
        if self.nc > 0:
            f_sym = sp.Matrix(holonomic_constraints)
            J_sym = f_sym.jacobian(q)
            # Check if constraints depend on time t
            self._con_has_t = any(f.has(t_sym) for f in holonomic_constraints)
            con_args = tuple(q) + (t_sym,) if self._con_has_t else tuple(q)
            self.J_func = sp.lambdify(con_args, J_sym, modules="numpy")
            self.f_func = sp.lambdify(con_args, f_sym, modules="numpy")
            if self._con_has_t:
                # Explicit time derivative ∂f/∂t: f_dot = J qd + f_t is the
                # total derivative, which Baumgarte needs for time-varying
                # constraints (moving anchors, moving rails).  Computed by
                # central differences: the symbolic Derivative of a
                # dynamicsymbol cannot be lambdified by NumPyPrinter.
                self._f_t_step = 1e-6
        else:
            self.J_func = None
            self.f_func = None

        # External generalized forces: list of (coord_index, sympy_expression)
        if external_forces:
            self._ext_force_funcs = []
            for idx, expr in external_forces:
                f = sp.lambdify((t_sym,), expr, modules="numpy")
                self._ext_force_funcs.append((idx, f))
        else:
            self._ext_force_funcs = None

    def _solve_dae(self, t, state):
        """Solve the DAE at (t, state): return (qdd, multipliers)."""
        q = state[:self.nq]
        qd = state[self.nq:]

        M_args = list(q) + [t] if self._M_has_t else list(q)
        M_full = np.asarray(self.M_full_func(*M_args))
        F_full = np.asarray(self.F_full_func(*(list(q) + list(qd) + [t]))).ravel()

        # Add external generalized forces to the nq dynamic rows
        if self._ext_force_funcs:
            for idx, f_func in self._ext_force_funcs:
                F_full[self.nq + idx] += float(f_func(t))

        if self.nc > 0:
            con_args = list(q) + [t] if self._con_has_t else list(q)
            J_val = np.asarray(self.J_func(*con_args))
            f_val = np.asarray(self.f_func(*con_args)).ravel()
            f_dot = J_val @ qd
            if self._f_t_step > 0.0:
                h = self._f_t_step
                f_p = np.asarray(self.f_func(*(list(q) + [t + h]))).ravel()
                f_m = np.asarray(self.f_func(*(list(q) + [t - h]))).ravel()
                f_dot = f_dot + (f_p - f_m) / (2.0 * h)
            cons_start = 2 * self.nq
            F_full[cons_start:] = F_full[cons_start:] - 2.0 * self.BAUMGARTE_ALPHA * f_dot - self.BAUMGARTE_BETA**2 * f_val

        sol = np.linalg.lstsq(M_full, F_full, rcond=None)[0].ravel()
        qdd = sol[self.nq:2 * self.nq]
        lams = sol[2 * self.nq:2 * self.nq + self.nc]
        return qdd, lams

    def rhs(self, t, state):
        qdd, _ = self._solve_dae(t, state)
        return np.concatenate([state[self.nq:], qdd])

    def multipliers(self, t, state):
        """Evaluate Lagrange multipliers (constraint forces) at (t, state)."""
        _, lams = self._solve_dae(t, state)
        return lams

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
        state0 = np.concatenate([q0, qd0])

        result = solve_ivp(
            self.rhs,
            t_span,
            state0,
            method=method,
            events=events,
            atol=atol,
            rtol=rtol,
            max_step=0.02,
        )

        return result
