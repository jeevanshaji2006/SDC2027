"""
sim_check.py  -  does the planned slalom + hurdle line survive real-world errors?  (runs on your PC)

Simulates the SAME loop the STM32 will run (200 Hz):
    pose estimate (encoder speed + gyro heading)  ->  Ramsete  ->  v, omega  ->  skid-steer robot
with the errors that hurt on a real skid-steer chassis:
    - robot placed on the start line with a small position / heading error the estimator does not know about
    - gyro bias + noise, encoder scale error (these make the ESTIMATE drift)
    - yaw gain error (scrub) and sideways slide in turns (the estimator is BLIND to this)
Then it checks the TRUE path against the gate posts and hurdle strips, 300 times.

What it does NOT model: the speed loss while climbing a hurdle (that is the boost's job; test it on the robot),
and ToF/EKF corrections (add them later; this shows how much error they would have to fix).

Run:  python sim_check.py
"""
import math, random
import numpy as np
import sprint_config as C
import slalom_path as S

# Ramsete gains (WPILib-style)
B, ZETA = 2.0, 0.7
DT = 1.0 / C.CONTROL_HZ

# error model (1-sigma)  -  change these to match what you measure on the real robot
ERR = dict(start_x=0.010, start_y=0.020, start_th=math.radians(2.0),     # placement error on the line
           gyro_bias=math.radians(0.05), gyro_noise=0.010, gyro_scale=0.003,
           enc_scale=0.010, enc_noise=0.010,
           drive_gain=0.03, yaw_gain_mean=0.95, yaw_gain_sd=0.05,      # yaw gain <1 = scrub loses rotation
           slide_mean=0.030, slide_sd=0.020,                            # s : lateral slip = -slide*v*omega
           tau_v=0.10, tau_w=0.10)


def sinc(x):
    return 1.0 - x * x / 6.0 if abs(x) < 1e-4 else math.sin(x) / x


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ramsete(est, ref):
    x, y, th = est
    rx, ry, rth, rv, rw = ref
    dx, dy = rx - x, ry - y
    ex = math.cos(th) * dx + math.sin(th) * dy
    ey = -math.sin(th) * dx + math.cos(th) * dy
    et = wrap(rth - th)
    k = 2 * ZETA * math.sqrt(rw * rw + B * rv * rv)
    return rv * math.cos(et) + k * ex, rw + k * et + B * rv * sinc(et) * ey


def run_once(plan, rng, E=ERR):
    # reference resampled at the control rate, TABLE frame (start pose = 0,0,0)
    tc = np.arange(0.0, plan.t[-1], DT)
    R = np.stack([np.interp(tc, plan.t, plan.x) - C.START_X, np.interp(tc, plan.t, plan.y),
                  np.interp(tc, plan.t, plan.th), np.interp(tc, plan.t, plan.v), np.interp(tc, plan.t, plan.w)], axis=1)

    x, y, th = rng.gauss(0, E["start_x"]), rng.gauss(0, E["start_y"]), rng.gauss(0, E["start_th"])   # TRUE pose
    v = w = 0.0
    ex_, ey_, eth = 0.0, 0.0, 0.0                       # ESTIMATE (firmware assumes a perfect start)
    kv = 1 + rng.gauss(0, E["drive_gain"])
    kw = rng.gauss(E["yaw_gain_mean"], E["yaw_gain_sd"])
    slide = max(0.0, rng.gauss(E["slide_mean"], E["slide_sd"]))
    bias = rng.gauss(0, E["gyro_bias"])
    sf, es = rng.gauss(0, E["gyro_scale"]), rng.gauss(0, E["enc_scale"])

    out = np.empty((len(tc), 3))
    for i in range(len(tc)):
        out[i] = (x, y, th)
        vc, wc = ramsete((ex_, ey_, eth), R[i])
        vc = min(max(vc, 0.0), C.V_MAX * 1.15)
        wc = min(max(wc, -4.0), 4.0)
        v += (vc * kv - v) * DT / E["tau_v"]
        w += (wc * kw - w) * DT / E["tau_w"]
        vlat = -slide * v * w                           # body-frame sideways slide (outward in a turn)
        x += (v * math.cos(th) - vlat * math.sin(th)) * DT
        y += (v * math.sin(th) + vlat * math.cos(th)) * DT
        th += w * DT
        # estimator: gyro heading + encoder speed
        eth += (w * (1 + sf) + bias + rng.gauss(0, E["gyro_noise"])) * DT
        venc = v * (1 + es) + rng.gauss(0, E["enc_noise"])
        ex_ += venc * math.cos(eth) * DT
        ey_ += venc * math.sin(eth) * DT
    return tc, out


def evaluate_run(tc, tr):
    xl = tr[:, 0] + C.START_X                           # back to lane coordinates for the geometry checks
    gc = S.gate_clearances(xl, tr[:, 1], tr[:, 2])
    hm = S.hurdle_margins(xl, tr[:, 1], tr[:, 2])
    return dict(gate=gc, hmargin=[a for a, _ in hm], hhead=[b for _, b in hm],
                T=S.finish_time(xl, tc) if xl[-1] >= C.FINISH_X + C.ROBOT_L / 2 else float("inf"))


def monte_carlo(plan, n=300, seed=1, keep=40):
    rng = random.Random(seed)
    res, paths = [], []
    for k in range(n):
        tc, tr = run_once(plan, rng)
        res.append(evaluate_run(tc, tr))
        if k < keep:
            paths.append(tr)
    return res, paths


def summarize(res, plan):
    n = len(res)
    gate_ok = np.array([[g >= 0 for g in r["gate"]] for r in res])
    gate_min = np.array([r["gate"] for r in res])
    hm = np.array([r["hmargin"] for r in res])
    hh = np.degrees(np.array([r["hhead"] for r in res]))
    T = np.array([r["T"] for r in res])
    hurdle_ok = (hm >= 0)
    all_ok = gate_ok.all(axis=1) & hurdle_ok.all(axis=1) & np.isfinite(T)
    print(f"\n=== Monte Carlo: {n} simulated runs of the planned line ===")
    for i in range(gate_ok.shape[1]):
        print(f"  Gate {i+1}:   passed without touching a post in {100*gate_ok[:, i].mean():5.1f} % of runs; "
              f"clearance p5/median = {100*np.percentile(gate_min[:, i], 5):5.1f} / {100*np.median(gate_min[:, i]):5.1f} cm "
              f"(planned {100*plan.metrics['gate_clear'][i]:.1f} cm)")
    for i in range(hm.shape[1]):
        print(f"  Hurdle {i+1}: crossed over (body inside the 60 cm strip) in {100*hurdle_ok[:, i].mean():5.1f} % of runs; "
              f"lateral margin p5/median = {100*np.percentile(hm[:, i], 5):5.1f} / {100*np.median(hm[:, i]):5.1f} cm; "
              f"heading on step p95 = {np.percentile(hh[:, i], 95):4.1f} deg")
    fin = T[np.isfinite(T)]
    print(f"  Finish time: median {np.median(fin):.2f} s, p95 {np.percentile(fin, 95):.2f} s (reference plan {plan.metrics['finish_time']:.2f} s)")
    print(f"  CLEAN RUNS (no gate hit, both hurdles crossed over): {100*all_ok.mean():.1f} %")
    return all_ok.mean()


def plot_paths(plan, paths, png="sim_paths.png"):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, a = plt.subplots(figsize=(12, 3.8))
    a.add_patch(Rectangle((0, -C.LANE_W / 2), C.FINISH_X, C.LANE_W, fc="#eef3fb", ec="#345"))
    for gx, gy in C.GATES:
        for s in (-1, 1):
            a.plot(gx, gy + s * C.GATE_OPENING / 2, "s", c="#1f5fa8", ms=7)
    for hx, hy in C.HURDLES:
        a.add_patch(Rectangle((hx - .01, hy - C.HURDLE_W / 2), .02, C.HURDLE_W, fc="#c58b00"))
    for tr in paths:
        a.plot(tr[:, 0] + C.START_X, tr[:, 1], c="#d33", lw=.6, alpha=.5)
    a.plot(plan.x, plan.y, c="k", lw=1.2, label="planned")
    a.axvline(C.FINISH_X, c="#c58b00", lw=2); a.set_aspect("equal"); a.legend()
    a.set_title("Closed-loop paths (centre of robot), 40 simulated runs")
    fig.tight_layout(); fig.savefig(png, dpi=110); plt.close(fig)


def clean_fraction(res):
    ok = [all(g >= 0 for g in r["gate"]) and all(h >= 0 for h in r["hmargin"]) and math.isfinite(r["T"]) for r in res]
    return sum(ok) / len(ok)


def ablation(plan, n=150):
    """Which error source ruins the dead-reckoned line?  Re-run with one source removed at a time."""
    scen = [
        ("baseline (all errors)", {}),
        ("start heading error 2.0 -> 0.5 deg", dict(start_th=math.radians(0.5))),
        ("start heading error 2.0 -> 0.0 deg", dict(start_th=0.0)),
        ("no sideways slide in turns", dict(slide_mean=0.0, slide_sd=0.0)),
        ("no start-placement error at all", dict(start_x=0.0, start_y=0.0, start_th=0.0)),
        ("no placement error AND no slide", dict(start_x=0.0, start_y=0.0, start_th=0.0, slide_mean=0.0, slide_sd=0.0)),
    ]
    print("\n=== Ablation: what limits a dead-reckoned (encoder + gyro only) run? ===")
    for name, ch in scen:
        E = dict(ERR); E.update(ch)
        rng = random.Random(7)
        res = [evaluate_run(*run_once(plan, rng, E)) for _ in range(n)]
        print(f"  {name:<40s} clean runs: {100*clean_fraction(res):5.1f} %")


if __name__ == "__main__":
    plan = S.plan_best(verbose=False)
    res, paths = monte_carlo(plan)
    summarize(res, plan)
    plot_paths(plan, paths)
    ablation(plan)
    print("\nWrote sim_paths.png")
