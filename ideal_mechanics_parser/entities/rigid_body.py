from .base import Node


class RigidBody(Node):
    def __init__(self, id, init_state=None, params=None):
        super().__init__(id, "RigidBody", init_state, params)
        self.m = None
        self.I = None
        self.idx = None
        self.x_sym = None
        self.y_sym = None
        self.theta_sym = None
        self.vx_sym = None
        self.vy_sym = None
        self.omega_sym = None
