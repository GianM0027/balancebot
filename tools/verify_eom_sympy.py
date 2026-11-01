"""Independent symbolic check of the planar equations of motion.

Builds T and V from the body positions, applies Euler-Lagrange with SymPy,
extracts M(q), h(q, qd), B and compares them with the hand derivation.
`symbolic_model()` is also used by tests/test_dynamics.py.

Usage:  python tools/verify_eom_sympy.py
"""
import sympy as sp


def symbolic_model():
    """Derive M, h, B symbolically.

    Returns:
        dict with M, h, B (SymPy matrices in plain symbols x, th, xd, thd)
        and the parameter symbols.
    """
    t = sp.symbols("t")
    m_w, m_b, I_w, I_b, l, r, g, tau = sp.symbols("m_w m_b I_w I_b l r g tau", positive=True)
    x = sp.Function("x")(t)
    th = sp.Function("theta")(t)
    q = sp.Matrix([x, th])
    qd = q.diff(t)
    qdd = qd.diff(t)

    # kinematics (world frame, z = 0 at the axle)
    p_b = sp.Matrix([x + l * sp.sin(th), l * sp.cos(th)])   # body CoM
    v_b = p_b.diff(t)
    phi_dot = qd[0] / r                                      # rolling: x = r*phi

    # energies
    T = (sp.Rational(1, 2) * m_w * qd[0] ** 2
         + sp.Rational(1, 2) * I_w * phi_dot ** 2
         + sp.Rational(1, 2) * m_b * (v_b.T * v_b)[0]
         + sp.Rational(1, 2) * I_b * qd[1] ** 2)
    V = m_b * g * p_b[1]
    L = T - V

    # generalised forces from virtual work: dW = tau*(dphi - dtheta)
    Q = sp.Matrix([tau / r, -tau])

    eom = sp.Matrix([
        sp.expand(sp.diff(L.diff(qd[i]), t) - L.diff(q[i]) - Q[i]) for i in range(2)
    ])
    M = eom.jacobian(qdd)
    B = -eom.jacobian(sp.Matrix([tau]))
    h = sp.simplify(eom - M * qdd + B * tau)

    # replace functions of time with plain symbols
    xs, ths, xds, thds = sp.symbols("x th xd thd")
    sub = {qd[0]: xds, qd[1]: thds, x: xs, th: ths}
    M, h, B = (sp.simplify(E.subs(sub)) for E in (M, h, B))
    T_plain = sp.simplify(T.subs(sub))
    return {
        "M": M, "h": h, "B": B, "T": T_plain,
        "state": (xs, ths, xds, thds),
        "params": {"m_w": m_w, "m_b": m_b, "I_w": I_w, "I_b": I_b, "l": l, "r": r, "g": g},
    }


def main():
    mdl = symbolic_model()
    M, h, B, T = mdl["M"], mdl["h"], mdl["B"], mdl["T"]
    P = mdl["params"]
    _, th, xd, thd = mdl["state"]
    m_w, m_b, I_w, I_b, l, r, g = (P[k] for k in ("m_w", "m_b", "I_w", "I_b", "l", "r", "g"))

    M_hand = sp.Matrix([[m_w + I_w / r**2 + m_b, m_b * l * sp.cos(th)],
                        [m_b * l * sp.cos(th),   I_b + m_b * l**2]])
    h_hand = sp.Matrix([-m_b * l * sp.sin(th) * thd**2, -m_b * g * l * sp.sin(th)])
    B_hand = sp.Matrix([1 / r, -1])

    checks = {
        "M": sp.simplify(M - M_hand) == sp.zeros(2, 2),
        "h": sp.simplify(h - h_hand) == sp.zeros(2, 1),
        "B": sp.simplify(B - B_hand) == sp.zeros(2, 1),
        "T = 1/2 qd^T M qd": sp.simplify(sp.hessian(T, (xd, thd)) - M_hand) == sp.zeros(2, 2),
    }
    sp.init_printing(use_unicode=True)
    print("M(q) =");       sp.pprint(M)
    print("\nh(q, qd) ="); sp.pprint(h)
    print("\nB =");        sp.pprint(B)
    print()
    for name, ok in checks.items():
        print(f"[{'OK' if ok else 'MISMATCH'}] {name}")
    return all(checks.values())


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
