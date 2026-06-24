from .base import Edge


class FixedCoordinate(Edge):
    def __init__(self, id, from_id, to_id=None, params=None):
        super().__init__(id, "FixedCoordinate", from_id, to_id, params)
        self.coord = params.get("coord", "x")
        self.value = float(params.get("value", 0.0))
