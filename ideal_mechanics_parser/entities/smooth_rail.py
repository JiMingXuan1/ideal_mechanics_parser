from .base import Edge


class SmoothRail(Edge):
    def __init__(self, id, from_id, to_id, params=None):
        super().__init__(id, "SmoothRail", from_id, to_id, params)
        self.expr_str = params.get("expr", "y")
