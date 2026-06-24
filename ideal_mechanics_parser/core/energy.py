import sympy as sp


GAUSS_G = 0.000295912208  # k^2, Gaussian gravitational constant squared
EPS = 0.001               # softening length (AU units)


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

    # N-body gravitational potential (Gauss G = k^2)
    grav = env.get("gravitation", {})
    if grav.get("enabled"):
        V_G = assemble_gravitational_potential(points, bodies, sm, grav)
        V += V_G

    return T, V


def assemble_gravitational_potential(points, rigid_bodies, sm, grav_cfg):
    """N-body pairwise gravitational potential V_G.

    V_G = -G * sum_{i<j} m_i * m_j / sqrt(|r_i - r_j|^2 + epsilon^2)

    Uses Gaussian gravitational constant (k^2) when G is not specified.
    """
    G = float(grav_cfg.get("G", GAUSS_G))
    eps = float(grav_cfg.get("epsilon", EPS))
    eps_sq = eps * eps

    all_bodies = list(points)
    if rigid_bodies:
        all_bodies.extend(rigid_bodies)

    V_G = 0
    for i in range(len(all_bodies)):
        for j in range(i + 1, len(all_bodies)):
            a = all_bodies[i]
            b = all_bodies[j]
            dx = a.x_sym - b.x_sym
            dy = a.y_sym - b.y_sym
            d_sq = dx**2 + dy**2
            V_G += -G * a.m * b.m / sp.sqrt(d_sq + eps_sq)

    return sp.simplify(V_G)
