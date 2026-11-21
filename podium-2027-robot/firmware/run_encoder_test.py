"""Encoder + speed-estimator check at the Sprint cruise speed (open loop, 1 motor)."""
import math, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import params as P
from battery import Battery
from pwm import PWMTimer
from driver import Driver
from motor import Motor
from encoder import Encoder, SpeedEstimator

V_CRUISE = 1.10                                            # m/s, planner cruise cap (sprint_config.V_MAX)
RPM_CRUISE = V_CRUISE / P.R_WHEEL * 60 / (2 * math.pi)     # output-shaft rpm

def run(duty, windows=(1, 5, 10), start_count=0, t_end=1.2):
    bat, tim, drv, m = Battery(), PWMTimer(), Driver(), Motor()
    enc = Encoder(start_count=start_count)
    est = {w: SpeedEstimator(enc, window=w) for w in windows}
    tick_every = round(1 / (P.CTRL_HZ * P.DT))
    ib = 0.0
    log = {"t": [], "true": [], **{w: [] for w in windows}, "reg": []}
    for k in range(int(t_end / P.DT)):
        t = k * P.DT
        d, du, _ = tim.update(duty if t >= 0.1 else 0.0)
        i = m.step(drv.source_voltage(d, du, bat.voltage(ib)), drv.r_series, P.DT)
        ib = du * abs(i)
        enc.update(m.w, P.DT)
        if k % tick_every == 0:
            for w, e in est.items(): e.tick()
            log["t"].append(t); log["true"].append(m.output_rpm); log["reg"].append(enc.register)
            for w, e in est.items(): log[w].append(e.output_rpm)
    return {k: np.array(v) for k, v in log.items()}

# find the open-loop duty that gives the cruise speed
lo, hi = 0.0, P.DUTY_CAP
for _ in range(25):
    mid = (lo + hi) / 2
    r = run(mid, windows=(1,), t_end=1.0)
    lo, hi = (mid, hi) if r["true"][-1] < RPM_CRUISE else (lo, mid)
duty = (lo + hi) / 2

cpm_ms = P.ENC_CPR * RPM_CRUISE * P.N / 60 / 1000
print(f"encoder (ASSUMED): {P.ENC_PPR} PPR x4 = {P.ENC_CPR} counts per motor rev, {P.ENC_BITS}-bit counter")
print(f"cruise {V_CRUISE} m/s = {RPM_CRUISE:.0f} rpm at the wheel  (open-loop duty {duty:.3f}, no load)")
print(f"-> {cpm_ms:.2f} counts per 1 ms tick at cruise\n")

r = run(duty)
ss = (r["t"] > 0.9) & (r["t"] < 1.2)
print("steady-state speed error vs the true speed (output-shaft rpm):")
print(f"  {'window':>9} {'one count =':>12} {'rms error':>10} {'max error':>10}")
for w in (1, 5, 10):
    e = r[w][ss] - r["true"][ss]
    res = 60 / P.ENC_CPR / (w / P.CTRL_HZ) / P.N
    print(f"  {w:>6} ms {res:>9.1f} rpm {np.sqrt(np.mean(e**2)):>7.1f} rpm {np.abs(e).max():>7.1f} rpm")

# wrap test: start the 16-bit counter 40 counts below its roll-over point
rw = run(duty, windows=(10,), start_count=(1 << P.ENC_BITS) - 40)
r0 = run(duty, windows=(10,), start_count=0)
print(f"\nwrap test: register crossed {P.ENC_BITS}-bit roll-over: {bool((np.diff(rw['reg']) < 0).any())}, "
      f"max |speed difference| vs no-wrap run: {np.abs(rw[10] - r0[10]).max():.2e} rpm")

# plot
fig, ax = plt.subplots(1, 2, figsize=(11, 4))
for w, c in ((1, "tab:red"), (5, "tab:orange"), (10, "tab:green")):
    ax[0].plot(r["t"], r[w], c, lw=0.9, label=f"{w} ms window")
ax[0].plot(r["t"], r["true"], "k", lw=2, label="true"); ax[0].set_xlim(0.7, 1.2)
ax[0].set_ylim(RPM_CRUISE * 0.4, RPM_CRUISE * 1.6)
ax[0].set_title(f"Speed seen by the controller at {V_CRUISE} m/s"); ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("rpm (wheel)")
ax[0].legend(); ax[0].grid(True)
ax[1].plot(rw["t"], rw["reg"], ".", ms=2); ax[1].set_title("16-bit counter register across the wrap")
ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("TIMx->CNT"); ax[1].grid(True)
plt.tight_layout(); plt.savefig("encoder_test.png", dpi=120)
