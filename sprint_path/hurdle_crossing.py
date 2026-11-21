"""
hurdle_crossing.py  -  SECTION 2: crossing the two 5 cm hurdles without going around (-10) or stalling (Reset -15)

What it does
  1. Takes the plan from slalom_path.py and finds, for each hurdle, WHERE on the path
        - the front wheels touch the hurdle        (x_front)
        - the rear wheels have cleared it          (x_rear)
        - the feed-forward boost must start        (x_front - BOOST_LEAD_M, i.e. before contact)
        - the heading-hold must be active          (the straight window the planner built)
  2. Verifies the reference is really straight and steady-speed over each hurdle.
  3. Does a static wheel-vs-step check (h/R and the friction needed to climb) so you can warn the
     mechanical side early.
  4. Writes sprint_hurdle.h: boost windows (as estimated-X thresholds AND table ticks), boost level and
     ramps, the IMU-pitch BACKUP trigger and the stall-detector settings for the STM32 firmware.
  5. Writes sprint_hurdle_zones.png.

Why the boost is triggered by POSITION, not by pitch
  By the time the BMI088 pitch passes 8 deg the front wheels are already on the step and speed has
  started to fall. So the firmware starts the boost ~10 cm before contact using the estimated X, and uses
  pitch > PITCH_ON_DEG only as a backup (pose error) together with a motor-stall detector.

Run:  python hurdle_crossing.py        (run slalom_path.py first, or let this import and re-plan)
"""
import math
import numpy as np
import sprint_config as C
import slalom_path as S

# ------------------------------------------------------------------ tuning values (start here, tune on the robot)
BOOST_LEAD_M      = 0.10     # start boosting this far BEFORE the front wheels touch
BOOST_DUTY        = 0.20     # extra PWM duty (0..1) added on top of the speed feed-forward   TUNE
BOOST_RAMP_IN_MS  = 30
BOOST_RAMP_OUT_MS = 80
BOOST_TAIL_M      = 0.03     # keep boosting a little after the rear wheels clear
HEADING_HOLD_TOL  = math.radians(5.0)   # firmware: hold heading to the reference, error limit

PITCH_ON_DEG      = 8.0      # backup trigger (BMI088 chassis pitch)
PITCH_OFF_DEG     = 3.0
STALL_SPEED_FRAC  = 0.30     # stalled if measured speed < 30 % of reference ...
STALL_TIME_MS     = 80       # ... for this long while inside a boost window
STALL_PUSH_DUTY   = 0.35     # extra duty during the stall push
STALL_PUSH_MS     = 150


# ------------------------------------------------------------------ helpers
def _tick_at_x(plan, x, last_inside=False):
    """First tick with X >= x; with last_inside=True, the last tick with X <= x (used for window ends)."""
    if last_inside:
        return int(np.clip(np.searchsorted(plan.x, x, side="right") - 1, 0, len(plan.x) - 1))
    return int(np.clip(np.searchsorted(plan.x, x), 0, len(plan.x) - 1))


def hurdle_zones(plan):
    """One dict per hurdle, all X in LANE coordinates and in TABLE coordinates (start pose = 0)."""
    zones = []
    knots = {(k.kind, k.idx): k for k in plan.knots}
    for i, (hx, hy) in enumerate(C.HURDLES):
        x_front = hx - C.FRONT_REACH
        x_rear = hx + C.REAR_REACH
        x_on = x_front - BOOST_LEAD_M
        x_off = x_rear + BOOST_TAIL_M
        hold_in = [k for k in plan.knots if k.kind == "hurdle_in" and abs(k.x - (hx - C.FRONT_REACH - C.HURDLE_MARGIN)) < 1e-6][0]
        hold_out = [k for k in plan.knots if k.kind == "hurdle_out" and abs(k.x - (hx + C.REAR_REACH + C.HURDLE_MARGIN)) < 1e-6][0]
        a, b = _tick_at_x(plan, x_on), _tick_at_x(plan, x_off, True)
        h0, h1 = _tick_at_x(plan, hold_in.x), _tick_at_x(plan, hold_out.x, True)
        seg = slice(h0, h1 + 1)
        zones.append(dict(
            idx=i, hx=hx, hy=hy, x_front=x_front, x_rear=x_rear, x_on=x_on, x_off=x_off,
            boost_start=a, boost_end=b, hold_start=h0, hold_end=h1,
            v_min=float(plan.v[seg].min()), v_max=float(plan.v[seg].max()),
            max_head=float(np.abs(plan.th[seg]).max()), max_omega=float(np.abs(plan.w[seg]).max()),
            y_dev=float(np.abs(plan.y[seg] - plan.y[seg].mean()).max()),
            t_contact=float(plan.t[_tick_at_x(plan, x_front)]),
            v_contact=float(plan.v[_tick_at_x(plan, x_front)]),
        ))
    return zones


def step_climb_check(R=C.WHEEL_R, h=C.HURDLE_H, mass=C.MASS_KG, mu=C.MU_TYRE, front_share=0.5):
    """
    Static, zero-speed check for a wheel of radius R climbing a step of height h (edge contact).
    Horizontal force needed at the wheel centre = W_wheel * tan(phi), tan(phi) = sqrt(h(2R-h))/(R-h).
    The rear wheels have to supply it through friction  ->  needed mu = front_share * tan(phi).
    Conservative: real robots add approach speed (momentum) and tread bite on the edge.
    """
    if h >= R:
        return dict(h_over_R=h / R, tan_phi=float("inf"), mu_needed=float("inf"), ok=False,
                    msg="step is as tall as the wheel radius - cannot climb from a standing start")
    tan_phi = math.sqrt(h * (2 * R - h)) / (R - h)
    mu_need = front_share * tan_phi
    ok = mu_need <= mu
    # kinetic energy the robot carries into the hurdle at the planned speed vs. the potential energy to raise the front half by h
    return dict(h_over_R=h / R, tan_phi=tan_phi, mu_needed=mu_need, ok=ok,
                msg=("static check passed" if ok else
                     "NOT met statically: crossing relies on approach speed + tread grip. Rule of thumb h/R <= 0.5 "
                     f"(R >= {2*h*100:.0f} cm). Verify on the bench before trusting the boost alone."))


def momentum_check(v, mass=C.MASS_KG, h=C.HURDLE_H, front_share=0.5):
    """Kinetic energy at contact vs. the energy to lift the front half of the robot by h (very rough)."""
    ke = 0.5 * mass * v ** 2
    pe = front_share * mass * 9.81 * h
    return ke, pe


# ------------------------------------------------------------------ export
def export(zones, plan, header="sprint_hurdle.h"):
    with open(header, "w") as f:
        f.write("/* AUTO-GENERATED by hurdle_crossing.py - do not edit by hand */\n#pragma once\n#include <stdint.h>\n")
        f.write("/* x thresholds are in TABLE coordinates (robot start pose = 0), compare with the EKF/odometry X estimate.\n")
        f.write("   *_tick values are indices into SPRINT_TRAJ (10 ms per tick). */\n")
        f.write("typedef struct {\n  float x_boost_on, x_boost_off;     /* start/stop feed-forward boost (estimated X) */\n"
                "  uint16_t boost_start_tick, boost_end_tick;\n"
                "  uint16_t hold_start_tick, hold_end_tick;      /* heading-hold window (straight reference) */\n"
                "  float hurdle_x, hurdle_y;                     /* table coords, for logging */\n} SprintHurdleZone;\n")
        f.write(f"#define SPRINT_N_HURDLES {len(zones)}\n")
        f.write("static const SprintHurdleZone SPRINT_HURDLE[SPRINT_N_HURDLES] = {\n")
        for z in zones:
            f.write(f"  {{ {z['x_on']-C.START_X:.4f}f, {z['x_off']-C.START_X:.4f}f, {z['boost_start']}, {z['boost_end']}, "
                    f"{z['hold_start']}, {z['hold_end']}, {z['hx']-C.START_X:.4f}f, {z['hy']:.4f}f }},\n")
        f.write("};\n")
        f.write(f"#define SPRINT_BOOST_DUTY          {BOOST_DUTY:.3f}f   /* extra duty on top of feed-forward */\n")
        f.write(f"#define SPRINT_BOOST_RAMP_IN_MS    {BOOST_RAMP_IN_MS}\n#define SPRINT_BOOST_RAMP_OUT_MS   {BOOST_RAMP_OUT_MS}\n")
        f.write(f"#define SPRINT_HEADING_HOLD_TOL_RAD {HEADING_HOLD_TOL:.4f}f\n")
        f.write(f"/* backup triggers */\n#define SPRINT_PITCH_ON_DEG        {PITCH_ON_DEG:.1f}f\n#define SPRINT_PITCH_OFF_DEG       {PITCH_OFF_DEG:.1f}f\n")
        f.write(f"#define SPRINT_STALL_SPEED_FRAC    {STALL_SPEED_FRAC:.2f}f\n#define SPRINT_STALL_TIME_MS       {STALL_TIME_MS}\n")
        f.write(f"#define SPRINT_STALL_PUSH_DUTY     {STALL_PUSH_DUTY:.2f}f\n#define SPRINT_STALL_PUSH_MS       {STALL_PUSH_MS}\n")


def plot(zones, plan, png="sprint_hurdle_zones.png"):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    ax[0].plot(plan.t, plan.v, c="#1f5fa8", label="reference v [m/s]")
    ax[1].plot(plan.t, np.degrees(plan.th), c="#d33", label="reference heading [deg]")
    ax[1].plot(plan.t, plan.y * 100, c="#555", lw=.8, label="reference Y [cm]")
    for z in zones:
        t_on, t_off = plan.t[z["boost_start"]], plan.t[z["boost_end"]]
        t_h0, t_h1 = plan.t[z["hold_start"]], plan.t[z["hold_end"]]
        for a in ax:
            a.axvspan(t_h0, t_h1, color="#9ac" , alpha=.25)
            a.axvspan(t_on, t_off, color="#c58b00", alpha=.30)
        ax[0].text(t_on, ax[0].get_ylim()[1] * .92, f" H{z['idx']+1}: boost {t_on:.2f}-{t_off:.2f} s", fontsize=9)
        ax[0].axvline(z["t_contact"], c="k", ls=":", lw=1)
    ax[0].legend(loc="lower right"); ax[1].legend(loc="lower right")
    ax[1].set_xlabel("t [s]  (gold = boost window, blue = heading-hold window, dotted = front-wheel contact)")
    for a in ax: a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(png, dpi=110); plt.close(fig)


def report(zones, plan):
    print("\n=== Hurdle crossing plan ===")
    for z in zones:
        steady = (z["v_max"] - z["v_min"]) / max(z["v_max"], 1e-6)
        print(f"Hurdle {z['idx']+1} at X={z['hx']:.2f} m (lane), Y={z['hy']:+.2f} m")
        print(f"   front wheels touch at X={z['x_front']:.2f} m (t={z['t_contact']:.2f} s, v={z['v_contact']:.2f} m/s); "
              f"rear wheels clear at X={z['x_rear']:.2f} m")
        print(f"   boost window  X {z['x_on']:.2f} -> {z['x_off']:.2f} m   (ticks {z['boost_start']} -> {z['boost_end']})")
        print(f"   hold window   ticks {z['hold_start']} -> {z['hold_end']}:  ref heading max {math.degrees(z['max_head']):.2f} deg, "
              f"|omega| max {z['max_omega']:.3f} rad/s, lateral wander {100*z['y_dev']:.2f} cm")
        print(f"   speed in hold window {z['v_min']:.2f}-{z['v_max']:.2f} m/s  ({100*steady:.0f} % spread)  "
              f"{'STEADY' if steady < 0.10 else 'NOT STEADY - lengthen the straight run before the hurdle'}")
    ck = step_climb_check()
    print(f"\nWheel check: R={C.WHEEL_R*100:.1f} cm (placeholder), step {C.HURDLE_H*100:.0f} cm -> h/R = {ck['h_over_R']:.2f}, "
          f"tan(phi) = {ck['tan_phi']:.2f}, friction needed {ck['mu_needed']:.2f} vs mu = {C.MU_TYRE:.2f}")
    print("   ->", ck["msg"])
    v = min(z["v_contact"] for z in zones)
    ke, pe = momentum_check(v)
    print(f"   momentum at contact ({v:.2f} m/s): KE = {ke:.1f} J vs ~{pe:.1f} J to raise the front half by 5 cm (rough)")
    print(f"\nFirmware rules exported: boost +{BOOST_DUTY:.2f} duty from X_on to X_off (ramp {BOOST_RAMP_IN_MS}/{BOOST_RAMP_OUT_MS} ms); "
          f"backup: pitch > {PITCH_ON_DEG:.0f} deg or stall (<{100*STALL_SPEED_FRAC:.0f} % speed for {STALL_TIME_MS} ms) -> "
          f"+{STALL_PUSH_DUTY:.2f} duty for {STALL_PUSH_MS} ms")


if __name__ == "__main__":
    plan = S.plan_best(verbose=False)
    zones = hurdle_zones(plan)
    report(zones, plan)
    export(zones, plan); plot(zones, plan)
    print("\nWrote sprint_hurdle.h, sprint_hurdle_zones.png")
