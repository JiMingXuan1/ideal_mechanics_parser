import numpy as np
from .exceptions import ProjectionError


def project_initial_state(q0, qd0, constraints_func, jacobian_func, max_iter=50, tol=1e-12, extra_args=None):
    q = q0.copy()
    args = extra_args or []

    for iteration in range(max_iter):
        f_val = constraints_func(*q, *args).ravel()
        error = np.linalg.norm(f_val)
        if error < tol:
            return q

        J = jacobian_func(*q, *args)
        J_pinv = np.linalg.pinv(J)
        delta = J_pinv @ f_val
        q = q - delta

    raise ProjectionError(
        f"Newton-Raphson did not converge after {max_iter} iterations. "
        f"Final residual: {np.linalg.norm(constraints_func(*q, *args)):.2e}. "
        "Initial topology is invalid!"
    )
