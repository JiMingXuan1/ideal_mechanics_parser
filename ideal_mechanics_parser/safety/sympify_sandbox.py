import io
import tokenize

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

SAFE_FUNCTIONS = {
    "sin", "cos", "tan", "exp", "sqrt", "Abs", "log",
    "asin", "acos", "atan", "sinh", "cosh", "tanh",
}

_ALLOWED_OP_TOKENS = {"+", "-", "*", "**", "/", "(", ")", ","}

_SKIP_TOKEN_TYPES = {
    tokenize.ENCODING, tokenize.NEWLINE, tokenize.NL,
    tokenize.ENDMARKER, tokenize.INDENT, tokenize.DEDENT,
}


def _validate_tokens(expr_str, allowed_names):
    """Reject any token outside {numbers, whitelisted names, arithmetic ops}.

    This runs BEFORE sp.sympify: SymPy's sympify can eval arbitrary Python
    (e.g. ``__import__('os').system(...)``), so the raw string must be
    constrained to pure arithmetic first.
    """
    try:
        tokens = list(tokenize.generate_tokens(io.StringIO(expr_str).readline))
    except (tokenize.TokenError, IndentationError, SyntaxError) as e:
        raise SecurityError(f"Invalid expression syntax: {e}")

    prev = None
    for tok in tokens:
        if tok.type in _SKIP_TOKEN_TYPES:
            continue
        if tok.type == tokenize.NUMBER:
            prev = tok
            continue
        if tok.type == tokenize.NAME:
            if tok.string not in allowed_names:
                raise SecurityError(
                    f"Disallowed identifier '{tok.string}'. Allowed: "
                    f"{', '.join(sorted(allowed_names))} and math functions.")
            prev = tok
            continue
        if tok.type == tokenize.OP:
            if tok.string not in _ALLOWED_OP_TOKENS:
                raise SecurityError(f"Disallowed operator '{tok.string}'")
            if tok.string == "(" and prev is not None and prev.type == tokenize.NAME \
                    and prev.string not in SAFE_FUNCTIONS:
                raise SecurityError(
                    f"'{prev.string}' is not a callable math function")
            prev = tok
            continue
        raise SecurityError(f"Disallowed token '{tok.string}'")


def safe_sympify(expr_str, extra_locals=None):
    if not isinstance(expr_str, str) or not expr_str.strip():
        raise SecurityError("Empty expression")

    locals_dict = dict(SAFE_LOCALS)
    if extra_locals:
        locals_dict.update(extra_locals)

    allowed_names = set(locals_dict.keys())
    _validate_tokens(expr_str, allowed_names)

    expr = sp.sympify(expr_str, locals=locals_dict, evaluate=False)

    # Symbols that may legitimately appear: whitelisted names plus the free
    # symbols of any symbolic values passed in (e.g. the dynamicsymbols x0,
    # rx0, rt0 substituted for x/y/t on a SmoothRail or moving anchor).
    allowed_symbols = set(allowed_names)
    for v in locals_dict.values():
        if isinstance(v, sp.Basic):
            allowed_symbols |= {str(s) for s in v.free_symbols}

    for s in expr.free_symbols:
        if str(s) not in allowed_symbols:
            raise SecurityError(
                f"Disallowed symbol detected: '{s}'. Only t, x, y, m, g and "
                f"math functions are permitted.")

    return expr
