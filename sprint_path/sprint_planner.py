#!/usr/bin/env python3
"""
ASME PODIUM CY2027 - The Sprint: offline trajectory planner.

Frame: X = forward along the lane (0 at the start line), Y = left of the lane
centreline, theta = 0 means "facing down the lane".  Units: m, s, rad.

What it does
  1. Builds waypoints from the course config (gates, hurdles, finish).
  2. Joins them with quintic Hermite splines (position, heading, curvature set
     at every knot) with a STRAIGHT zone around each hurdle.
  3. Builds a speed profile (accel, lateral-accel and hurdle speed limits).
  4. Samples at 10 ms -> [x, y, theta, v, omega] and checks gate/hurdle rules.
  5. (--sim) tracks it with Ramsete under scrub + gyro-bias noise.
  6. Writes trajectory.h for the STM32 and a plot.

All gate/hurdle numbers are ESTIMATES read from Figure 2 (not to scale).
Edit the CONFIG block after the Q&A answer or after measuring the real field.
"""
import math
import argparse
import numpy as np

# ------------------------------ CONFIG -------------------------------------
ROBOT_W, ROBOT_L = 0.42, 0.45          # m  (must pass a 0.60 m gate opening)
GATE_OPEN = 0.60
GATE_TOL = (GATE_OPEN - ROBOT_W) / 2   # allowed centre error at a gate (0.09 m)
LANE_LEN = 7.0

GATES = [(1.0, +0.5), (3.0, -0.5), (4.2, +0.5), (5.9, -0.5)]   # (X, Y centre)
HURDLES = [(2.0, -0.5), (4.8, +0.5)]   # (X, Y centre), 5 cm tall, 0.6 m wide, same side as nearest gate
APPROACH = 0.20                        # straight run-in before a hurdle (m)
EXIT = 0.20                            # straight run-out after a hurdle (m)

V_MAX = 1.0                            # m/s cruise
V_HURDLE = 0.6                         # m/s through a hurdle (steady, no braking)
A_MAX = 1.2                            # m/s^2 longitudinal
A_LAT = 1.5                            # m/s^2 lateral (skid-steer scrub limit)
MAX_GATE_HEADING = math.radians(35)
TAN_SCALE = 0.55                       # spline tangent length / chord (smaller = less overshoot)
HURDLE_HALF = 0.30                     # hurdle half-width (m), centred on the crossing line
DT = 0.010                             # table step (s)
# ---------------------------------------------------------------------------


def build_waypoints():
    """Return list of (x, y, heading or None)."""
    ev = [("G", x, y) for x, y in GATES] + [("H", x, y) for x, y in HURDLES]
    ev.sort(key=lambda e: e[1])
    w = [(0.0, 0.0, 0.0)]
    for kind, x, y in ev:
        if kind == "H":                       # straight, heading 0, same Y
            w += [(x - APPROACH, y, 0.0), (x, y, 0.0), (x + EXIT, y, 0.0)]
        else:
            w.append((x, y, None))            # heading chosen below
    w.append((LANE_LEN + ROBOT_L + 0.05, w[-1][1], 0.0))   # rear fully past line
    out = []
    last_gate_x = max(x for x, _ in GATES)
    for i, (x, y, th) in enumerate(w):
        if th is None:
            if x == last_gate_x:
                th = 0.0
            else:
                dx, dy = w[i + 1][0] - w[i - 1][0], w[i + 1][1] - w[i - 1][1]
                th = float(np.clip(math.atan2(dy, dx), -MAX_GATE_HEADING, MAX_GATE_HEADING))
        out.append((x, y, th))
    return out


def quintic_hermite(p0, v0, a0, p1, v1, a1, u):
    u = u[:, None]
    h0 = 1 - 10*u**3 + 15*u**4 - 6*u**5
    h1 = u - 6*u**3 + 8*u**4 - 3*u**5
    h2 = 0.5*u**2 - 1.5*u**3 + 1.5*u**4 - 0.5*u**5
    h3 = 0.5*u**3 - u**4 + 0.5*u**5
    h4 = -4*u**3 + 7*u**4 - 3*u**5
    h5 = 10*u**3 - 15*u**4 + 6*u**5
    return h0*p0 + h1*v0 + h2*a0 + h3*a1 + h4*v1 + h5*p1


def dense_path(wps, n=400):
    pts = []
    for (x0, y0, t0), (x1, y1, t1) in zip(wps[:-1], wps[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        p0, p1 = np.array([x0, y0]), np.array([x1, y1])
        v0 = TAN_SCALE * d * np.array([math.cos(t0), math.sin(t0)])
        v1 = TAN_SCALE * d * np.array([math.cos(t1), math.sin(t1)])
        z = np.zeros(2)
        pts.append(quintic_hermite(p0, v0, z, p1, v1, z, np.linspace(0, 1, n, endpoint=False)))
    pts.append(np.array([[wps[-1][0], wps[-1][1]]]))
    P = np.vstack(pts)
    ds = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))
    s = np.concatenate([[0], np.cumsum(ds)])
    dx, dy = np.gradient(P[:, 0], s), np.gradient(P[:, 1], s)
    th = np.unwrap(np.arctan2(dy, dx))
    kappa = np.gradient(th, s)
    kappa = np.convolve(kappa, np.ones(15) / 15, mode="same")
    return s, P[:, 0], P[:, 1], th, kappa


def speed_profile(s, x, kappa):
    vlim = np.minimum(V_MAX, np.sqrt(A_LAT / (np.abs(kappa) + 1e-9)))
    for hx, _ in HURDLES:
        m = (x > hx - APPROACH - 0.05) & (x < hx + EXIT + 0.05)
        vlim[m] = np.minimum(vlim[m], V_HURDLE)
    v = vlim.copy()
    v[0] = v[-1] = 0.0
    for i in range(1, len(v)):                       # forward: accel limit
        v[i] = min(v[i], math.sqrt(v[i-1]**2 + 2*A_MAX*(s[i]-s[i-1])))
    for i in range(len(v) - 2, -1, -1):              # backward: decel limit
        v[i] = min(v[i], math.sqrt(v[i+1]**2 + 2*A_MAX*(s[i+1]-s[i])))
    return v


def build_table():
    wps = build_waypoints()
    s, x, y, th, k = dense_path(wps)
    v = speed_profile(s, x, k)
    vv = np.maximum(v, 1e-3)
    t = np.concatenate([[0], np.cumsum(2*np.diff(s) / (vv[:-1] + vv[1:]))])
    tt = np.arange(0, t[-1], DT)
    st = np.interp(tt, t, s)
    f = lambda a: np.interp(st, s, a)
    tab = np.column_stack([f(x), f(y), f(th), f(v), f(v) * f(k)])
    return wps, tab


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ramsete(pose, ref, b=2.0, zeta=0.7):
    x, y, th = pose
    xr, yr, thr, vr, wr = ref
    dx, dy = xr - x, yr - y
    ex = math.cos(th)*dx + math.sin(th)*dy
    ey = -math.sin(th)*dx + math.cos(th)*dy
    et = wrap(thr - th)
    k = 2*zeta*math.sqrt(wr*wr + b*vr*vr)
    sinc = 1 - et*et/6 if abs(et) < 1e-4 else math.sin(et)/et
    return vr*math.cos(et) + k*ex, wr + k*et + b*vr*sinc*ey


def check(tab):
    print(f"Table: {len(tab)} rows x 5 floats = {len(tab)*5*4/1024:.1f} KB flash")
    t_fin = np.argmax(tab[:, 0] >= LANE_LEN + ROBOT_L/2) * DT
    print(f"Finish (rear past line) at {t_fin:.2f} s -> score ~ min(100, 100 - {t_fin:.1f} + 10) "
          f"= {min(100, 100 - t_fin + 10):.0f}")
    print(f"Peak heading {math.degrees(np.max(np.abs(tab[:,2]))):.0f} deg, "
          f"peak |omega| {np.max(np.abs(tab[:,4])):.2f} rad/s, peak v {tab[:,3].max():.2f} m/s")
    for i, (gx, gy) in enumerate(GATES, 1):
        j = np.argmax(tab[:, 0] >= gx)
        print(f"  Gate {i}: plan Y={tab[j,1]:+.3f} (centre {gy:+.2f}, tol +-{GATE_TOL:.2f}) "
              f"t={j*DT:.2f}s v={tab[j,3]:.2f}")
    for i, (hx, hy) in enumerate(HURDLES, 1):
        m = (tab[:, 0] > hx - APPROACH) & (tab[:, 0] < hx + EXIT)
        print(f"  Hurdle {i}: heading {math.degrees(np.abs(tab[m,2]).max()):.1f} deg, "
              f"|omega| {np.abs(tab[m,4]).max():.2f}, v {tab[m,3].min():.2f}-{tab[m,3].max():.2f}")


def simulate(tab, runs=200, seed=1):
    """Ramsete + unicycle with yaw scrub, start offset and gyro-bias heading error."""
    rng = np.random.default_rng(seed)
    ok = 0
    worst = 0.0
    for _ in range(runs):
        x, y = rng.normal(0, 0.02), rng.normal(0, 0.02)
        th = math.radians(rng.normal(0, 2))
        scrub = rng.uniform(0.85, 0.97)               # achieved/commanded yaw rate
        bias = math.radians(rng.normal(0, 0.1))       # gyro bias after calibration (rad/s)
        xs, ys = [], []
        for k in range(len(tab) + 100):
            ref = tab[min(k, len(tab) - 1)]
            t = k * DT
            est = (x + rng.normal(0, 0.003), y + rng.normal(0, 0.003), th + bias*t)
            v, w = ramsete(est, ref)
            th += w * scrub * DT
            x += v * math.cos(th) * DT
            y += v * math.sin(th) * DT
            xs.append(x); ys.append(y)
        xs, ys = np.array(xs), np.array(ys)
        errs = [abs(ys[np.argmax(xs >= gx)] - gy) for gx, gy in GATES]
        worst = max(worst, max(errs))
        ok += all(e <= GATE_TOL for e in errs)
    print(f"Simulation ({runs} runs, scrub 0.85-0.97, start offset, gyro bias 0.1 deg/s sigma): "
          f"{ok}/{runs} pass all gates, worst gate error {worst*100:.1f} cm")


def export_header(tab, path="trajectory.h"):
    with open(path, "w") as f:
        f.write("/* Generated by sprint_planner.py - do not edit. Columns: x, y, theta, v, omega */\n")
        f.write("#ifndef TRAJECTORY_H\n#define TRAJECTORY_H\n")
        f.write(f"#define TRAJ_N {len(tab)}\n#define TRAJ_DT {DT:.3f}f\n")
        f.write(f"#define N_HURDLES {len(HURDLES)}\n")
        f.write("static const float HURDLE_X[N_HURDLES] = {" +
                ", ".join(f"{h[0]:.3f}f" for h in HURDLES) + "};\n")
        f.write("static const float TRAJ[TRAJ_N][5] = {\n")
        for r in tab:
            f.write("  {" + ", ".join(f"{c:.4f}f" for c in r) + "},\n")
        f.write("};\n#endif\n")
    print("Wrote", path)


def plot(wps, tab, path="sprint_path.png"):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed - skipping plot")
        return
    fig, ax = plt.subplots(2, 1, figsize=(11, 6), gridspec_kw={"height_ratios": [2, 1]})
    ax[0].plot(tab[:, 0], tab[:, 1], "k", lw=1.5, label="planned centre")
    for gx, gy in GATES:
        for sgn in (-1, 1):
            ax[0].plot([gx, gx], [gy + sgn*GATE_OPEN/2, gy + sgn*(GATE_OPEN/2 + 0.12)], "b", lw=4)
        ax[0].plot([gx, gx], [gy - GATE_TOL, gy + GATE_TOL], "g", lw=2)
    for hx, hy in HURDLES:
        ax[0].plot([hx, hx], [hy - HURDLE_HALF, hy + HURDLE_HALF], color="orange", lw=6)
    ax[0].axvline(0, color="gray"); ax[0].axvline(LANE_LEN, color="red")
    ax[0].set_ylim(-1, 1); ax[0].set_aspect("equal"); ax[0].set_ylabel("Y left (m)")
    ax[0].set_title("Sprint path: blue = gate posts, green = allowed centre band, orange = hurdles")
    t = np.arange(len(tab)) * DT
    ax[1].plot(t, tab[:, 3], label="v (m/s)"); ax[1].plot(t, tab[:, 4], label="omega (rad/s)")
    ax[1].set_xlabel("time (s)"); ax[1].legend(); ax[1].grid(True)
    fig.tight_layout(); fig.savefig(path, dpi=110)
    print("Wrote", path)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true", help="run Ramsete Monte-Carlo check")
    args = ap.parse_args()
    wps, tab = build_table()
    check(tab)
    if args.sim:
        simulate(tab)
    export_header(tab)
    plot(wps, tab)
