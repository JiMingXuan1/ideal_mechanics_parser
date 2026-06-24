import sympy as sp


def assemble_energy(points, edges, env, sm, rigid_bodies=None):
    T = 0
    V = 0

    view_plane = env.get("view_plane", "XY")
    gravity = float(env.get("gravity", 9.81))

    for p in points:
        T += sp.Rational(1, 2) * p.m * (p.vx_sym**2 + p.vy_sym**2)
        if view_plane == "XZ":
            V += p.m * gravity * p.y_sym

    bodies = rigid_bodies if rigid_bodies is not None else sm.rigid_bodies
    for b in bodies:
        T += sp.Rational(1, 2) * b.m * (b.vx_sym**2 + b.vy_sym**2)
        T += sp.Rational(1, 2) * b.I * b.omega_sym**2
        if view_plane == "XZ":
            V += b.m * gravity * b.y_sym

    for e in edges:
        if e.type == "IdealSpring":
            a = e.from_node
            b = e.to_node
            dx = a.x_sym - b.x_sym
            dy = a.y_sym - b.y_sym
            dist_sq = dx**2 + dy**2
            V += sp.Rational(1, 2) * e.k * (sp.sqrt(dist_sq) - e.l0)**2

    return T, V
