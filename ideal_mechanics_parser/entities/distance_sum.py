from .base import Edge


class DistanceSum(Edge):
    def __init__(self, id, from_id, to_id=None, params=None):
        super().__init__(id, "DistanceSum", from_id, to_id, params)
        self.via_id = params.get("via_id", None)
        self.length = float(params.get("length", 10.0))
