import sympy as sp
from core.exceptions import SecurityError

SAFE_LOCALS = {
    "t": sp.Symbol("t"),
    "sin": sp.sin,
    "cos": sp.cos,
    "tan": sp.tan,
    "pi": sp.pi,
    "exp": sp.exp,
    "sqrt": sp.sqrt,
    "Abs": sp.Abs,
    "log": sp.log,
    "asin": sp.asin,
    "acos": sp.acos,
    "atan": sp.atan,
    "sinh": sp.sinh,
    "cosh": sp.cosh,
    "tanh": sp.tanh,
}


def safe_sympify(expr_str, extra_locals=None):
    locals_dict = dict(SAFE_LOCALS)
    if extra_locals:
        locals_dict.update(extra_locals)

    expr = sp.sympify(expr_str, locals=locals_dict, evaluate=False)

    allowed_symbols = set(locals_dict.keys())
    for s in expr.free_symbols:
        s_str = str(s)
        if s_str not in allowed_symbols and not s_str.startswith("x") and not s_str.startswith("y"):
            raise SecurityError(f"Disallowed symbol detected: '{s_str}'. Only t, x, y and math functions are permitted.")

    return expr
