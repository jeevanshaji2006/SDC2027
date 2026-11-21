"""
slalom_path.py  -  SECTION 1: slalom path through the four Gates (offline, runs on your PC)

What it does
  1. Builds waypoints at the gate centres, straight "hurdle windows", the finish and a stop point.
  2. Joins them with a piecewise QUINTIC (5th order) path Y(X): position, slope (heading)
     and curvature are all continuous, so the robot never has to stop and re-align.
  3. Gives the path a speed profile (curvature, accel and hurdle-speed limits).
  4. Samples it every 10 ms -> [x, y, theta, v, omega]  (the table the STM32 Ramsete loop reads).
  5. Checks the plan against the real geometry: body-to-gate-post clearance, hurdle lateral
     margin, heading over hurdles, yaw rate, predicted finish time and score.
  6. Writes sprint_traj.h (C array for flash), sprint_traj.csv and sprint_path.png.

Run:  python slalom_path.py
"""
import math
from dataclasses import dataclass
import numpy as np
import sprint_config as C

# ------------------------------------------------------------------ quintic Hermite  Y(X)
def _quintic(h, y0, y1, s0, s1, c0, c1):
    """Coefficients of Y(tau), tau=(X-X0)/h, matching Y, dY/dX, d2Y/dX2 at both ends."""
    a0, a1, a2 = y0, s0 * h, 0.5 * c0 * h * h
    p = (y1 - y0) - a1 - a2
    q = s1 * h - a1 - 2 * a2
    r = c1 * h * h - 2 * a2
    a3 = 10 * p - 4 * q + 0.5 * r
    a4 = -15 * p + 7 * q - r
    a5 = 6 * p - 3 * q + 0.5 * r
    return np.array([a0, a1, a2, a3, a4, a5])


@dataclass
class Knot:
    x: float
    y: float
    s: float = 0.0      # dY/dX  (= tan(heading))
    c: float = 0.0      # d2Y/dX2
    kind: str = ""
    idx: int = -1


class Path:
    """Piecewise quintic Y(X) over a sorted knot list."""
    def __init__(self, knots):
        self.k = knots
        self.xs = np.array([k.x for k in knots])
        assert np.all(np.diff(self.xs) > 1e-3), "knot X values must be strictly increasing"
        self.co = [_quintic(self.xs[i + 1] - self.xs[i], knots[i].y, knots[i + 1].y,
                            knots[i].s, knots[i + 1].s, knots[i].c, knots[i + 1].c)
                   for i in range(len(knots) - 1)]

    def eval(self, X):
        X = np.atleast_1d(np.asarray(X, float))
        seg = np.clip(np.searchsorted(self.xs, X, side="right") - 1, 0, len(self.co) - 1)
        Y, Yp, Ypp = np.empty_like(X), np.empty_like(X), np.empty_like(X)
        for i in np.unique(seg):
            m = seg == i
            h = self.xs[i + 1] - self.xs[i]
            t = (X[m] - self.xs[i]) / h
            a = self.co[i]
            Y[m]   = a[0] + t * (a[1] + t * (a[2] + t * (a[3] + t * (a[4] + t * a[5]))))
            Yp[m]  = (a[1] + t * (2 * a[2] + t * (3 * a[3] + t * (4 * a[4] + t * 5 * a[5])))) / h
            Ypp[m] = (2 * a[2] + t * (6 * a[3] + t * (12 * a[4] + t * 20 * a[5]))) / h ** 2
        return Y, Yp, Ypp


# ------------------------------------------------------------------ waypoints
def x_stop():
    """Where the reference path ends: rear edge clears the finish line, plus a short run-out."""
    return C.FINISH_X + C.ROBOT_L / 2 + C.OVERRUN_M


def lateral_tolerance():
    """How far the body centre may sit off an opening/hurdle centre at zero heading."""
    return min((C.GATE_OPENING - C.ROBOT_W) / 2, (C.HURDLE_W - C.ROBOT_W) / 2)


def build_knots(gate_scale):
    cut = min(C.CORNER_CUT, lateral_tolerance())
    toward_centre = lambda y: y - math.copysign(cut, y)     # aim slightly toward the lane centre
    pre = C.FRONT_REACH + C.HURDLE_MARGIN
    post = C.REAR_REACH + C.HURDLE_MARGIN

    kn = [Knot(C.START_X, 0.0, 0.0, 0.0, "start")]
    for i, (gx, gy) in enumerate(C.GATES):
        kn.append(Knot(gx, toward_centre(gy), 0.0, 0.0, "gate", i))
    for i, (hx, hy) in enumerate(C.HURDLES):
        y = toward_centre(hy)
        kn.append(Knot(hx - pre, y, 0.0, 0.0, "hurdle_in", i))     # straight, heading 0, no curvature
        kn.append(Knot(hx + post, y, 0.0, 0.0, "hurdle_out", i))
    y_last = toward_centre(C.GATES[-1][1])           # stay on the last gate's line after it
    kn.append(Knot(C.GATES[-1][0] + 0.45, y_last, 0.0, 0.0, "straighten"))
    kn.append(Knot(x_stop(), y_last, 0.0, 0.0, "stop"))
    kn.sort(key=lambda k: k.x)

    # gate heading = scaled secant between neighbouring knots (limited to +-35 deg)
    for j, k in enumerate(kn):
        if k.kind == "gate":
            a, b = kn[j - 1], kn[j + 1]
            slope = gate_scale * (b.y - a.y) / (b.x - a.x)
            k.s = float(np.clip(slope, -math.tan(math.radians(35)), math.tan(math.radians(35))))
    return kn


# ------------------------------------------------------------------ speed profile
def speed_profile(path, dx=0.002):
    X = np.arange(C.START_X, x_stop() + 1e-9, dx)
    Y, Yp, Ypp = path.eval(X)
    ds = np.sqrt(1 + Yp[:-1] ** 2) * dx
    kappa = Ypp / (1 + Yp ** 2) ** 1.5

    vlim = np.full_like(X, C.V_MAX)
    ak = np.maximum(np.abs(kappa), 1e-6)
    vlim = np.minimum(vlim, np.sqrt(C.A_LAT / ak))
    vlim = np.minimum(vlim, C.W_MAX / ak)
    for hx, _ in C.HURDLES:                        # steady, capped speed through every hurdle zone
        m = (X >= hx - C.FRONT_REACH - 0.15) & (X <= hx + C.REAR_REACH + 0.05)
        vlim[m] = np.minimum(vlim[m], C.V_HURDLE)
    vlim[0] = 0.0
    vlim[-1] = 0.0                                 # stop at the end of the run-out

    v = vlim.copy()
    for i in range(len(ds)):                       # forward pass  (acceleration limit)
        v[i + 1] = min(v[i + 1], math.sqrt(v[i] ** 2 + 2 * C.A_ACC * ds[i]))
    for i in range(len(ds) - 1, -1, -1):           # backward pass (braking limit)
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * C.A_DEC * ds[i]))

    t = np.zeros_like(X)
    for i in range(len(ds)):
        t[i + 1] = t[i] + 2 * ds[i] / max(v[i] + v[i + 1], 1e-6)
    return X, Y, Yp, kappa, v, t


# ------------------------------------------------------------------ plan = path + time table
@dataclass
class Plan:
    t: np.ndarray; x: np.ndarray; y: np.ndarray; th: np.ndarray
    v: np.ndarray; w: np.ndarray; kappa: np.ndarray
    knots: list; gate_scale: float; metrics: dict


def sample_plan(gate_scale):
    kn = build_knots(gate_scale)
    path = Path(kn)
    X, Y, Yp, kap, v, t = speed_profile(path)
    tt = np.arange(0.0, t[-1], C.TABLE_DT)
    xs = np.interp(tt, t, X)
    ys, yps, ypps = path.eval(xs)
    vs = np.interp(tt, t, v)
    kaps = ypps / (1 + yps ** 2) ** 1.5
    th = np.arctan(yps)
    plan = Plan(tt, xs, ys, th, vs, vs * kaps, kaps, kn, gate_scale, {})
    plan.metrics = evaluate(plan.x, plan.y, plan.th, plan.t, plan)
    return plan


# ------------------------------------------------------------------ geometry checks (also used by sim_check.py)
def footprint_corners(x, y, th):
    """(N,4,2) corners of the body rectangle ROBOT_L x ROBOT_W at each pose."""
    hl, hw = C.ROBOT_L / 2, C.ROBOT_W / 2
    loc = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    c, s = np.cos(th)[:, None], np.sin(th)[:, None]
    X = x[:, None] + c * loc[:, 0] - s * loc[:, 1]
    Y = y[:, None] + s * loc[:, 0] + c * loc[:, 1]
    return np.stack([X, Y], axis=2)


def post_clearance(x, y, th, px, py):
    """Signed distance from a thin post at (px,py) to the body rectangle (negative = inside = hit)."""
    c, s = np.cos(th), np.sin(th)
    u = c * (px - x) + s * (py - y)            # post in robot frame
    v = -s * (px - x) + c * (py - y)
    dx, dy = np.abs(u) - C.ROBOT_L / 2, np.abs(v) - C.ROBOT_W / 2
    outside = np.hypot(np.maximum(dx, 0), np.maximum(dy, 0))
    inside = np.maximum(dx, dy)
    return np.where((dx > 0) | (dy > 0), outside, inside)


def gate_clearances(x, y, th):
    out = []
    for gx, gy in C.GATES:
        near = np.abs(x - gx) < 0.6
        if not near.any():
            out.append(-1.0); continue
        c1 = post_clearance(x[near], y[near], th[near], gx, gy + C.GATE_OPENING / 2)
        c2 = post_clearance(x[near], y[near], th[near], gx, gy - C.GATE_OPENING / 2)
        out.append(float(min(c1.min(), c2.min())))
    return out


def hurdle_margins(x, y, th):
    """Lateral margin of the body inside the 60 cm hurdle while the body overlaps it, and max |heading| there."""
    res = []
    for hx, hy in C.HURDLES:
        corners = footprint_corners(x, y, th)
        overlap = (corners[:, :, 0].min(axis=1) <= hx) & (corners[:, :, 0].max(axis=1) >= hx)
        if not overlap.any():
            res.append((-1.0, math.pi)); continue
        cy = corners[overlap][:, :, 1]
        margin = float(np.min(np.minimum((hy + C.HURDLE_W / 2) - cy.max(axis=1),
                                         cy.min(axis=1) - (hy - C.HURDLE_W / 2))))
        # heading while the WHEELS are on the step (not just the body overlap)
        win = (x >= hx - C.FRONT_REACH) & (x <= hx + C.REAR_REACH)
        res.append((margin, float(np.max(np.abs(th[win]))) if win.any() else math.pi))
    return res


def finish_time(x, t):
    """Time when the rear edge of the body has fully crossed the finish line."""
    k = np.searchsorted(x, C.FINISH_X + C.ROBOT_L / 2)
    return float(t[min(k, len(t) - 1)])


def evaluate(x, y, th, t, plan=None):
    gc = gate_clearances(x, y, th)
    hm = hurdle_margins(x, y, th)
    T = finish_time(x, t)
    g1_time = float(t[np.searchsorted(x, C.GATES[0][0])])
    m = dict(gate_clear=gc,
             hurdle_margin=[a for a, _ in hm], hurdle_heading=[b for _, b in hm],
             finish_time=T, gate1_time=g1_time,
             score=min(100.0, max(0.0, 100 - T + (10 if g1_time < 30 else 0))))
    if plan is not None:
        m["max_heading_deg"] = float(np.degrees(np.max(np.abs(plan.th))))
        m["max_omega"] = float(np.max(np.abs(plan.w)))
        m["max_kappa"] = float(np.max(np.abs(plan.kappa)))
    m["ok"] = (min(gc) >= C.MIN_GATE_CLEAR and min(m["hurdle_margin"]) >= C.MIN_HURDLE_MARGIN
               and max(m["hurdle_heading"]) <= C.MAX_HURDLE_HEAD
               and (plan is None or m["max_omega"] <= C.W_MAX + 1e-6))
    m["worst_margin"] = min(min(gc), min(m["hurdle_margin"]))
    return m


# ------------------------------------------------------------------ choose the gate heading scale
def plan_best(scales=np.linspace(0.0, 1.6, 17), verbose=True):
    """Try several gate-heading scales; keep the plan with the largest worst-case margin (prefer 'ok')."""
    best = None
    for sc in scales:
        try:
            p = sample_plan(float(sc))
        except AssertionError:
            continue
        key = (p.metrics["ok"], p.metrics["worst_margin"])
        if verbose:
            print(f"  gate_scale {sc:4.2f}: worst margin {100*p.metrics['worst_margin']:6.1f} cm, "
                  f"max heading {p.metrics['max_heading_deg']:5.1f} deg, T {p.metrics['finish_time']:5.2f} s, "
                  f"{'OK' if p.metrics['ok'] else '--'}")
        if best is None or key > (best.metrics["ok"], best.metrics["worst_margin"]):
            best = p
    return best


# ------------------------------------------------------------------ export
def export(plan, header="sprint_traj.h", csv="sprint_traj.csv"):
    n = len(plan.t)
    with open(csv, "w") as f:
        f.write("t,x,y,theta,v,omega\n")
        for r in zip(plan.t, plan.x - C.START_X, plan.y, plan.th, plan.v, plan.w):
            f.write(",".join(f"{a:.5f}" for a in r) + "\n")
    m = plan.metrics
    with open(header, "w") as f:
        f.write("/* AUTO-GENERATED by slalom_path.py - do not edit by hand */\n")
        f.write("#pragma once\n/* Frame: X forward, Y LEFT, theta CCW from +X, origin = robot start pose.\n")
        f.write("   Columns: x[m] y[m] theta[rad] v[m/s] omega[rad/s]. Row k is at t = k*SPRINT_TRAJ_DT_S.\n")
        f.write("   Include from ONE .c file only (static const data lives in flash). */\n")
        f.write(f"#define SPRINT_TRAJ_DT_S   {C.TABLE_DT:.4f}f\n#define SPRINT_TRAJ_LEN    {n}\n")
        f.write(f"#define SPRINT_PRED_FINISH_S {m['finish_time']:.3f}f\n")
        f.write(f"#define SPRINT_N_GATES     {len(C.GATES)}\n")
        f.write("static const float SPRINT_GATE_CLEARANCE_M[SPRINT_N_GATES] = {"
                + ", ".join(f"{g:.4f}f" for g in m["gate_clear"]) + "};  /* planned body-to-post clearance = lateral tolerance budget */\n")
        f.write(f"static const float SPRINT_TRAJ[SPRINT_TRAJ_LEN][5] = {{\n")
        for r in zip(plan.x - C.START_X, plan.y, plan.th, plan.v, plan.w):   # table frame: start pose = (0,0)
            f.write("  {" + ", ".join(f"{a:.5f}f" for a in r) + "},\n")
        f.write("};\n")
    return n


def plot(plan, png="sprint_path.png"):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle
    fig, ax = plt.subplots(1, 2, figsize=(13, 6.5), gridspec_kw={"width_ratios": [1, 1.3]})
    a = ax[0]
    a.add_patch(Rectangle((0, -C.LANE_W / 2), C.FINISH_X, C.LANE_W, fc="#eef3fb", ec="#345"))
    for i, (gx, gy) in enumerate(C.GATES):
        for s in (-1, 1):
            a.plot(gx, gy + s * C.GATE_OPENING / 2, "s", c="#1f5fa8", ms=7)
        a.plot([gx, gx], [gy - .3, gy + .3], "--", c="#1f5fa8", lw=.8)
        a.text(gx, gy + .38 * math.copysign(1, gy), f"G{i+1}", ha="center", color="#1f5fa8")
    for i, (hx, hy) in enumerate(C.HURDLES):
        a.add_patch(Rectangle((hx - .01, hy - C.HURDLE_W / 2), .02, C.HURDLE_W, fc="#c58b00"))
        a.text(hx, hy - .42 * math.copysign(1, hy), f"H{i+1}", ha="center", color="#c58b00")
    a.axvline(C.FINISH_X, c="#c58b00", lw=2)
    a.plot(plan.x, plan.y, c="#d33", lw=1.6)
    for k in range(0, len(plan.t), 40):                      # footprint every 0.4 s
        co = footprint_corners(plan.x[k:k+1], plan.y[k:k+1], plan.th[k:k+1])[0]
        a.add_patch(Polygon(co, fc="none", ec="#555", lw=.6))
    a.set_aspect("equal"); a.set_xlabel("X forward [m]"); a.set_ylabel("Y left [m]")
    a.set_title("Top view: path + body footprint every 0.4 s")
    b = ax[1]
    b.plot(plan.t, plan.v, label="v [m/s]"); b.plot(plan.t, plan.w, label="omega [rad/s]")
    b.plot(plan.t, np.degrees(plan.th) / 20, label="heading [deg/20]")
    for hx, _ in C.HURDLES:
        t0 = plan.t[np.searchsorted(plan.x, hx - C.FRONT_REACH)]
        t1 = plan.t[np.searchsorted(plan.x, hx + C.REAR_REACH)]
        b.axvspan(t0, t1, color="#c58b00", alpha=.2)
    b.set_xlabel("t [s]"); b.legend(); b.grid(alpha=.3)
    b.set_title(f"Profile  (finish {plan.metrics['finish_time']:.2f} s, predicted score {plan.metrics['score']:.0f})")
    fig.tight_layout(); fig.savefig(png, dpi=110); plt.close(fig)


def report(plan):
    m = plan.metrics
    print(f"\nChosen gate_scale = {plan.gate_scale:.2f}   ->  {len(plan.t)} table rows "
          f"({len(plan.t)*5*4/1024:.1f} KB of flash)")
    for i, g in enumerate(m["gate_clear"]):
        print(f"  Gate {i+1}: body-to-post clearance {100*g:6.1f} cm   "
              f"{'OK' if g >= C.MIN_GATE_CLEAR else 'TOO TIGHT'}")
    for i, (a, b) in enumerate(zip(m["hurdle_margin"], m["hurdle_heading"])):
        print(f"  Hurdle {i+1}: lateral margin {100*a:6.1f} cm, heading on the step {math.degrees(b):4.1f} deg   "
              f"{'OK' if a >= C.MIN_HURDLE_MARGIN and b <= C.MAX_HURDLE_HEAD else 'CHECK'}")
    print(f"  max heading {m['max_heading_deg']:.1f} deg, max |omega| {m['max_omega']:.2f} rad/s, "
          f"tightest turn radius {1/m['max_kappa']:.2f} m")
    if 1 / m["max_kappa"] < 0.25:
        print("  WARNING: very tight turns (radius < 0.25 m). Skid-steer will scrub hard here. This comes from the short gate-to-hurdle")
        print("           distances read off Figure 2 - confirm the real spacing in the Q&A before trusting this plan.")
    print(f"  Gate 1 passed at t={m['gate1_time']:.2f} s (autonomous bonus if < 30 s)")
    print(f"  Predicted finish {m['finish_time']:.2f} s  ->  predicted Sprint score {m['score']:.0f}")
    print("  PLAN IS", "FEASIBLE with the current geometry/robot numbers" if m["ok"] else
          "NOT FEASIBLE - see the tight items above (change geometry, robot size or hurdle straight window)")


if __name__ == "__main__":
    C.check_board_a()
    print("Searching gate heading scale ...")
    best = plan_best()
    report(best)
    n = export(best); plot(best)
    print(f"\nWrote sprint_traj.h ({n} rows), sprint_traj.csv, sprint_path.png")
