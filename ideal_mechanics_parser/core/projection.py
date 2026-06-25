import numpy as np
from .exceptions import ProjectionError


def project_initial_state(q0, qd0, constraints_func, jacobian_func, max_iter=50, tol=1e-12, extra_args=None, context=None):
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

    from .exceptions import ProjectionError
    msg = (
        f"Newton-Raphson did not converge after {max_iter} iterations.\n"
        f"Final residual: {error:.2e}.\n"
        f"Constraint vector (first 3): {f_val[:3].tolist()}\n"
        f"Jacobian norm: {np.linalg.norm(J):.2e}\n"
    )
    if context:
        msg += f"Constraints: {context.get('expressions', [])}\n"
        msg += f"Initial position q0: {q0.tolist()}\n"
    msg += "Initial topology is invalid! Check that constraints reference dynamic bodies, not Anchors."
    raise ProjectionError(msg)
