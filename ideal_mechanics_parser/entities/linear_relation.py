from .base import Edge


class LinearRelation(Edge):
    def __init__(self, id, from_id, to_id=None, params=None):
        super().__init__(id, "LinearRelation", from_id, to_id, params)
        self.coeffs = params.get("coeffs", [1, -1, 0, 0])
        self.constant = float(params.get("constant", 0.0))
