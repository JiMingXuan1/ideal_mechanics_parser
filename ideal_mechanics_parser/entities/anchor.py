from .base import Node


class Anchor(Node):
    def __init__(self, id, init_pos, params=None):
        super().__init__(id, "Anchor", init_state={"x": init_pos[0], "y": init_pos[1]}, params=params)
        self.x0 = float(init_pos[0])
        self.y0 = float(init_pos[1])
