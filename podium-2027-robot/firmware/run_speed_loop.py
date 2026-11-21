"""Wheel speed loop (feedforward + PI + acceleration limit) on the full drive model.
4 identical wheels share one battery. Controller sees the ENCODER estimate, not the true speed."""
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import params as P
from battery import Battery
from pwm import PWMTimer
from driver import Driver
from motor import Motor
from encoder import Encoder, SpeedEstimator
from controller import SpeedLoop, ms_to_motor_w, motor_w_to_ms

# Controller's own model of the motor. Deliberately ~10 % off, as it would be on a real robot.
K_S  = 0.8 * (P.R_MOTOR + P.R_DRV) * P.I0   # V   (true friction voltage is a little higher)
K_V  = 0.9 * P.KE            # V per (rad/s)   (true value is KE)
K_P  = 5.2 * P.KE            # V per (rad/s) of error   (scaled with KE so the tuning carries over between buses)
K_I  = 157 * P.KE            # V per (rad/s * s)
EST_WINDOW = 10              # ms, speed-estimate window (from the encoder test)
V_CRUISE = 1.10              # m/s, planner cruise speed

def target(t):               # m/s
    if t < 0.1: return 0.0
    if t < 1.5: return V_CRUISE
    if t < 2.5: return 0.6
    return 0.0

def run(mode="closed", accel=2.0, r_batt=0.06, t_end=3.2):
    old = P.R_BATT; P.R_BATT = r_batt
    bat, drv = Battery(), Driver()
    tims = [PWMTimer() for _ in range(4)]
    ms = [Motor() for _ in range(4)]
    encs = [Encoder() for _ in range(4)]
    ests = [SpeedEstimator(e, window=EST_WINDOW) for e in encs]
    ctl = [SpeedLoop(K_S, K_V, K_P, K_I, accel) for _ in range(4)]
    tick = round(1 / (P.CTRL_HZ * P.DT))
    duty = [0.0] * 4; d_dir, d_mag = [1] * 4, [0.0] * 4
    ib, v_meas = 0.0, P.V_OC
    log = {k: [] for k in ("t", "tgt", "ref", "true", "est", "i", "vbus", "duty")}
    over, longest = 0.0, 0.0
    for k in range(int(t_end / P.DT)):
        t = k * P.DT
        vbus = bat.voltage(ib)
        if k % tick == 0:                                       # ---- control tick (1 kHz) ----
            v_meas += 0.2 * (vbus - v_meas)                     # ADC reading, lightly filtered
            for j in range(4):
                w_est = ests[j].tick()
                w_tgt = ms_to_motor_w(target(t))
                if mode == "closed":
                    duty[j] = ctl[j].update(w_tgt, w_est, v_meas)
                else:                                           # open loop: feedforward at full-battery voltage only
                    duty[j] = (K_S + K_V * w_tgt) / P.V_OC if w_tgt > 0 else 0.0
                d_dir[j], d_mag[j], _ = tims[j].update(duty[j])
        ib = 0.0
        for j in range(4):
            i = ms[j].step(drv.source_voltage(d_dir[j], d_mag[j], vbus), drv.r_series, P.DT)
            encs[j].update(ms[j].w, P.DT)
            ib += d_mag[j] * abs(i)
            if j == 0: i0 = i
        over = over + P.DT if abs(i0) >= 8.5 else 0.0
        longest = max(longest, over)
        if k % 20 == 0:
            for key, val in zip(log, (t, target(t), motor_w_to_ms(ctl[0].ref) if mode == "closed" else target(t),
                                      motor_w_to_ms(ms[0].w), motor_w_to_ms(ests[0].w), i0, vbus, d_mag[0])):
                log[key].append(val)
    P.R_BATT = old
    r = {k: np.array(v) for k, v in log.items()}
    r["longest_over_ms"] = longest * 1000
    return r

if __name__ == "__main__":
    base  = run("closed", accel=2.0)
    nolim = run("closed", accel=1e4)
    weakc = run("closed", accel=2.0, r_batt=4 * P.R_BATT)
    weako = run("open",   accel=2.0, r_batt=4 * P.R_BATT)
    healthy_o = run("open", accel=2.0)

    def at(r, tt): return r["true"][np.searchsorted(r["t"], tt)]
    def rise(r):
        s = r["true"] >= 0.9 * V_CRUISE
        return (r["t"][np.argmax(s)] - 0.1) * 1000 if s.any() else float("nan")
    ss = (base["t"] > 1.2) & (base["t"] < 1.5)
    print(f"Closed loop, healthy battery (R_batt {P.R_BATT} ohm), 2 m/s^2 limit:")
    print(f"  speed at 1.4 s        : {at(base,1.4):.3f} m/s   (target {V_CRUISE})")
    print(f"  time to 90 %          : {rise(base):.0f} ms")
    print(f"  overshoot             : {max(0, (base['true'].max()-V_CRUISE)/V_CRUISE*100):.1f} %")
    print(f"  steady error          : {np.mean(V_CRUISE-base['true'][ss])*1000:+.1f} mm/s")
    print(f"  peak motor current    : {base['i'].max():.1f} A")
    print(f"\nStall rule (>= 8.5 A for 50 ms) at launch:")
    print(f"  longest time >= 8.5 A, with 2 m/s^2 limit : {base['longest_over_ms']:.0f} ms")
    print(f"  longest time >= 8.5 A, no acceleration limit: {nolim['longest_over_ms']:.0f} ms")
    print(f"\nWeak battery (R_batt {4*P.R_BATT:.2f} ohm), speed at 1.4 s (target {V_CRUISE}):")
    print(f"  open loop  (feedforward only): {at(weako,1.4):.3f} m/s   (healthy battery: {at(healthy_o,1.4):.3f})")
    print(f"  closed loop (FF + PI)        : {at(weakc,1.4):.3f} m/s")

    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    ax[0,0].plot(base["t"], base["tgt"], "k--", label="target"); ax[0,0].plot(base["t"], base["ref"], "c:", label="ramped reference")
    ax[0,0].plot(base["t"], base["true"], label="true speed"); ax[0,0].plot(base["t"], base["est"], lw=0.7, label="encoder estimate")
    ax[0,0].set_title("Speed tracking (m/s)"); ax[0,0].legend(); ax[0,0].grid(True)
    ax[0,1].plot(weako["t"], weako["true"], label="open loop"); ax[0,1].plot(weakc["t"], weakc["true"], label="closed loop")
    ax[0,1].plot(base["t"], base["tgt"], "k--"); ax[0,1].set_title("Weak battery + model 10 % off: open vs closed (m/s)"); ax[0,1].legend(); ax[0,1].grid(True)
    ax[1,0].plot(base["t"], np.abs(base["i"]), label="2 m/s^2 limit"); ax[1,0].plot(nolim["t"], np.abs(nolim["i"]), label="no limit")
    ax[1,0].axhline(8.5, color="r", ls="--", label="stall threshold 8.5 A"); ax[1,0].set_xlim(0, 1.0)
    ax[1,0].set_title("Motor current at launch (A)"); ax[1,0].legend(); ax[1,0].grid(True)
    ax[1,1].plot(base["t"], base["duty"], label="duty"); ax2 = ax[1,1].twinx(); ax2.plot(base["t"], base["vbus"], "g", lw=0.8)
    ax[1,1].axhline(P.DUTY_CAP, color="r", ls=":"); ax[1,1].set_title("Duty (blue) and bus voltage (green)"); ax[1,1].grid(True)
    for a in ax.flat: a.set_xlabel("time (s)")
    plt.tight_layout(); plt.savefig("speed_loop.png", dpi=120)
