from .base import Edge


class SoftRope(Edge):
    def __init__(self, id, from_id, to_id, params=None):
        super().__init__(id, "SoftRope", from_id, to_id, params)
        self.length = float(params.get("length", 1.0)) if params else 1.0
        self._tight = False
