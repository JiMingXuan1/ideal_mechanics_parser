from .base import Edge


class HingeJoint(Edge):
    def __init__(self, id, from_id, to_id=None, params=None):
        super().__init__(id, "HingeJoint", from_id, to_id, params)
        self.pivot = params.get("pivot", [0.0, 0.0]) if params else [0.0, 0.0]
        self.world = params.get("world", None) if params else None
        self.pivot_b = params.get("pivot_b", None) if params else None
