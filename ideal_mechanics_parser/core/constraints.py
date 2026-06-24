import sympy as sp
from safety.sympify_sandbox import safe_sympify


def _world_pivot(body):
    """Return symbolic (x, y) of body's pivot point in world coordinates.

    For a MassPoint pivot is just (x_sym, y_sym).
    For a RigidBody with local pivot (u, v):
        x_world = x_sym + u*cos(theta) - v*sin(theta)
        y_world = y_sym + u*sin(theta) + v*cos(theta)
    For an Anchor, (x_sym, y_sym) are static floats.
    """
    if hasattr(body, "theta_sym") and body.theta_sym is not None:
        u, v = body.pivot_offset if hasattr(body, "pivot_offset") else (0.0, 0.0)
        ct = sp.cos(body.theta_sym)
        st = sp.sin(body.theta_sym)
        px = body.x_sym + u * ct - v * st
        py = body.y_sym + u * st + v * ct
        return px, py
    return body.x_sym, body.y_sym


def harvest_constraints(edges, sm):
    holonomic = []

    for e in edges:
        if e.type == "IdealRod":
            a = e.from_node
            b = e.to_node
            dx = a.x_sym - b.x_sym
            dy = a.y_sym - b.y_sym
            f = dx**2 + dy**2 - e.length**2
            holonomic.append(f)

        elif e.type == "SmoothRail":
            p = e.from_node
            local_vars = {"x": p.x_sym, "y": p.y_sym, "t": sm.t}
            f = safe_sympify(e.expr_str, local_vars)
            holonomic.append(f)

        elif e.type == "FixedCoordinate":
            p = e.from_node
            if e.coord == "x":
                f = p.x_sym - e.value
            elif e.coord == "y":
                f = p.y_sym - e.value
            else:
                raise ValueError(f"Unknown coord '{e.coord}' in FixedCoordinate")
            holonomic.append(f)

        elif e.type == "LinearRelation":
            ca, cb, cc, cd = e.coeffs[:4]
            if e.to_node is not None:
                f = (ca * e.from_node.x_sym + cb * e.from_node.y_sym
                     + cc * e.to_node.x_sym + cd * e.to_node.y_sym + e.constant)
            else:
                f = ca * e.from_node.x_sym + cb * e.from_node.y_sym + e.constant
            holonomic.append(f)

        elif e.type == "DistanceSum":
            p1 = e.from_node
            p2 = e.to_node
            if e.via_node is None:
                raise ValueError("DistanceSum requires via_node to be resolved")
            d1 = sp.sqrt((p1.x_sym - e.via_node.x_sym)**2 + (p1.y_sym - e.via_node.y_sym)**2)
            d2 = sp.sqrt((p2.x_sym - e.via_node.x_sym)**2 + (p2.y_sym - e.via_node.y_sym)**2)
            f = d1 + d2 - e.length
            holonomic.append(f)

        elif e.type == "AngleConstraint":
            p1 = e.from_node
            p2 = e.to_node
            sin_a = sp.sin(e.angle)
            cos_a = sp.cos(e.angle)
            f = (p2.x_sym - p1.x_sym) * sin_a - (p2.y_sym - p1.y_sym) * cos_a
            holonomic.append(f)

        elif e.type == "HingeJoint":
            a = e.from_node
            if hasattr(a, "theta_sym") and a.theta_sym is not None:
                a.pivot_offset = e.pivot
                ax, ay = _world_pivot(a)
            else:
                ax, ay = a.x_sym, a.y_sym

            if e.world is not None:
                fx = ax - float(e.world[0])
                fy = ay - float(e.world[1])
            elif e.to_node is not None:
                b = e.to_node
                if hasattr(b, "theta_sym") and b.theta_sym is not None:
                    b.pivot_offset = e.pivot_b if e.pivot_b is not None else [0.0, 0.0]
                    bx, by = _world_pivot(b)
                else:
                    bx, by = b.x_sym, b.y_sym
                fx = ax - bx
                fy = ay - by
            else:
                raise ValueError("HingeJoint needs either 'world' or 'to' node")

            holonomic.append(fx)
            holonomic.append(fy)

    return holonomic
