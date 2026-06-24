import math
from .base import Edge


class AngleConstraint(Edge):
    def __init__(self, id, from_id, to_id=None, params=None):
        super().__init__(id, "AngleConstraint", from_id, to_id, params)
        self.angle = float(params.get("angle", 0.0))
