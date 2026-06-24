import sympy as sp
from safety.sympify_sandbox import safe_sympify


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

    return holonomic
