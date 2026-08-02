import sympy as sp
from safety.sympify_sandbox import safe_sympify


def _rod_constraint(a_px, a_py, b_px, b_py, length):
    dx = a_px - b_px
    dy = a_py - b_py
    return dx**2 + dy**2 - length**2


def _world_pivot(body, pivot_offset=None):
    """Return symbolic (x, y) of body's pivot point in world coordinates.

    For a MassPoint pivot is just (x_sym, y_sym).
    For a RigidBody with local pivot (u, v):
        x_world = x_sym + u*cos(theta) - v*sin(theta)
        y_world = y_sym + u*sin(theta) + v*cos(theta)
    For an Anchor, (x_sym, y_sym) are static floats.
    """
    if hasattr(body, "theta_sym") and body.theta_sym is not None:
        u, v = pivot_offset if pivot_offset is not None else (0.0, 0.0)
        ct = sp.cos(body.theta_sym)
        st = sp.sin(body.theta_sym)
        px = body.x_sym + u * ct - v * st
        py = body.y_sym + u * st + v * ct
        return px, py
    return body.x_sym, body.y_sym


def _constraint_pair(e, sm):
    """Resolve (px, py) for from_node and (qx, qy) for to_node
    respecting RigidBody pivot offsets stored in edge params.
    """
    a = e.from_node
    from_pivot = getattr(e, "from_pivot", None) or e.params.get("from_pivot")
    a_px, a_py = _world_pivot(a, from_pivot)

    if e.to_node is not None:
        b = e.to_node
        to_pivot = getattr(e, "to_pivot", None) or e.params.get("to_pivot")
        b_px, b_py = _world_pivot(b, to_pivot)
    else:
        b_px = b_py = None

    return (a_px, a_py), (b_px, b_py)


def harvest_constraints(edges, sm):
    holonomic = []

    for e in edges:
        (a_px, a_py), (b_px, b_py) = _constraint_pair(e, sm)

        if e.type == "IdealRod":
            if b_px is None:
                raise ValueError("IdealRod requires 'to' node")
            holonomic.append(_rod_constraint(a_px, a_py, b_px, b_py, e.length))

        elif e.type == "SoftRope":
            if getattr(e, "_tight", False):
                if b_px is None:
                    raise ValueError("SoftRope requires 'to' node")
                holonomic.append(_rod_constraint(a_px, a_py, b_px, b_py, e.length))

        elif e.type == "SmoothRail":
            # The rail expression constrains a MassPoint to the curve f(x,y,t)=0.
            # If 'from' node has no DOFs (Anchor/static), find a dynamic body.
            from_node = e.from_node
            if hasattr(from_node, "idx") or (hasattr(from_node, "theta_sym") and from_node.theta_sym is not None):
                target_x, target_y = a_px, a_py
            else:
                # 'from' is static (Anchor). Try to find MassPoint via _rail_target.
                target = getattr(e, "_rail_target", None)
                if target is not None:
                    target_x, target_y = _world_pivot(target, None)
                else:
                    target_x, target_y = a_px, a_py
            local_vars = {"x": target_x, "y": target_y, "t": sm.t}
            f = safe_sympify(e.expr_str, local_vars)
            holonomic.append(f)

        elif e.type == "FixedCoordinate":
            if e.coord == "x":
                f = a_px - e.value
            elif e.coord == "y":
                f = a_py - e.value
            else:
                raise ValueError(f"Unknown coord '{e.coord}' in FixedCoordinate")
            holonomic.append(f)

        elif e.type == "LinearRelation":
            ca, cb, cc, cd = e.coeffs[:4]
            if b_px is not None:
                f = ca * a_px + cb * a_py + cc * b_px + cd * b_py + e.constant
            else:
                f = ca * a_px + cb * a_py + e.constant
            holonomic.append(f)

        elif e.type == "DistanceSum":
            p1x, p1y = a_px, a_py
            p2x, p2y = b_px, b_py
            if e.via_node is None:
                raise ValueError("DistanceSum requires via_node to be resolved")
            vx = e.via_node.x_sym if not hasattr(e.via_node, "theta_sym") or e.via_node.theta_sym is None \
                 else _world_pivot(e.via_node, [0, 0])[0]
            vy = e.via_node.y_sym if not hasattr(e.via_node, "theta_sym") or e.via_node.theta_sym is None \
                 else _world_pivot(e.via_node, [0, 0])[1]
            d1 = sp.sqrt((p1x - vx)**2 + (p1y - vy)**2)
            d2 = sp.sqrt((p2x - vx)**2 + (p2y - vy)**2)
            f = d1 + d2 - e.length
            holonomic.append(f)

        elif e.type == "AngleConstraint":
            if b_px is None:
                raise ValueError("AngleConstraint requires 'to' node")
            sin_a = sp.sin(e.angle)
            cos_a = sp.cos(e.angle)
            f = (b_px - a_px) * sin_a - (b_py - a_py) * cos_a
            holonomic.append(f)

        elif e.type == "HingeJoint":
            if e.world is not None:
                fx = a_px - float(e.world[0])
                fy = a_py - float(e.world[1])
            elif b_px is not None:
                fx = a_px - b_px
                fy = a_py - b_py
            else:
                raise ValueError("HingeJoint needs either 'world' or 'to' node")
            holonomic.append(fx)
            holonomic.append(fy)

    return holonomic
