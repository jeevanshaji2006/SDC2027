"""Battery -> PWM timer -> driver -> motor, open loop (no controller, no sensors)."""
import numpy as np
import matplotlib.pyplot as plt
import params as P
from battery import Battery
from pwm import PWMTimer
from driver import Driver
from motor import Motor

def duty_command(t):
    return P.DUTY_CAP if 0.1 <= t < 1.0 else 0.0     # step on at 0.1 s, off at 1.0 s

def run(n_motors, t_end=1.5, log_every=20):
    bat, timer, drv = Battery(), PWMTimer(), Driver()
    motors = [Motor() for _ in range(n_motors)]
    i_batt_prev = 0.0
    log = {k: [] for k in ("t", "v_bus", "i_motor", "i_batt", "rpm", "duty")}
    for k in range(int(t_end / P.DT)):
        t = k * P.DT
        v_bus = bat.voltage(i_batt_prev)                       # 1. battery (uses last step's current)
        direction, duty, _ = timer.update(duty_command(t))     # 2. timer / PWM
        v_src = drv.source_voltage(direction, duty, v_bus)     # 3. driver
        i_batt = 0.0
        for m in motors:                                       # 4. motors
            i = m.step(v_src, drv.r_series, P.DT)
            i_batt += duty * abs(i)                            # battery supplies current only while switch is on
        i_batt_prev = i_batt
        if k % log_every == 0:
            for key, val in zip(log, (t, v_bus, i, i_batt, motors[0].output_rpm, duty)):
                log[key].append(val)
    return {k: np.array(v) for k, v in log.items()}

if __name__ == "__main__":
    one, four = run(1), run(4)
    print(f"ARR = {P.ARR} ({P.ARR+1} steps), KE = {P.KE:.4f}, R_motor = {P.R_MOTOR:.2f} ohm, J = {P.J:.2e}")
    for name, r in (("1 motor ", one), ("4 motors", four)):
        print(f"{name}: peak motor current {r['i_motor'].max():5.1f} A | peak battery current {r['i_batt'].max():5.1f} A "
              f"| min bus voltage {r['v_bus'].min():5.2f} V | speed at 0.9 s {r['rpm'][np.searchsorted(r['t'], 0.9)]:5.0f} RPM")

    fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    for r, lab in ((one, "1 motor"), (four, "4 motors")):
        ax[0, 0].plot(r["t"], r["rpm"], label=lab)
        ax[0, 1].plot(r["t"], r["v_bus"], label=lab)
        ax[1, 0].plot(r["t"], r["i_motor"], label=lab)
        ax[1, 1].plot(r["t"], r["i_batt"], label=lab)
    for a, ttl, yl in ((ax[0, 0], "Wheel speed", "RPM (output shaft)"), (ax[0, 1], "Bus voltage", "V"),
                       (ax[1, 0], "Current in one motor", "A"), (ax[1, 1], "Total battery current", "A")):
        a.set_title(ttl); a.set_ylabel(yl); a.grid(True); a.legend()
    ax[1, 0].set_xlabel("time (s)"); ax[1, 1].set_xlabel("time (s)")
    plt.tight_layout(); plt.savefig("open_loop.png", dpi=120)
