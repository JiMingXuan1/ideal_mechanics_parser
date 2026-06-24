from .base import Edge


class IdealSpring(Edge):
    def __init__(self, id, from_id, to_id, params=None):
        super().__init__(id, "IdealSpring", from_id, to_id, params)
        self.k = float(params.get("k", 1.0))
        self.l0 = float(params.get("l0", 1.0))
