from .base import Edge


class IdealRod(Edge):
    def __init__(self, id, from_id, to_id, params=None):
        super().__init__(id, "IdealRod", from_id, to_id, params)
        self.length = float(params.get("length", 1.0))
