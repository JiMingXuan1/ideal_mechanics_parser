import sympy as sp


class Node:
    def __init__(self, id, type, init_state=None, params=None):
        self.id = id
        self.type = type
        self.init_state = init_state or {}
        self.params = params or {}


class Edge:
    def __init__(self, id, type, from_id, to_id, params=None):
        self.id = id
        self.type = type
        self.from_id = from_id
        self.to_id = to_id
        self.params = params or {}
        self.from_node = None
        self.to_node = None
