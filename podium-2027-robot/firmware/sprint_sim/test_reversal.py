"""Fix 2 check: DIR never changes while PWM is non-zero, even on a hard reversal."""
import numpy as np, params as P
from battery import Battery
from pwm import PWMTimer
from driver import Driver
from motor import Motor

def cmd(t):  return 0.7 if t < 0.6 else -0.7

bat, tim, drv, m = Battery(), PWMTimer(), Driver(), Motor()
ib, prev_dir, prev_cmp, bad, dead, i_min = 0.0, 1, 0, 0, 0, 0.0
for k in range(int(1.0 / P.DT)):
    t = k * P.DT
    d, duty, cmpv = tim.update(cmd(t))
    if d != prev_dir and prev_cmp != 0 or (d != prev_dir and cmpv != 0 and prev_cmp != 0): bad += 1
    if d != prev_dir: dead += 1
    prev_dir, prev_cmp = d, cmpv
    i = m.step(drv.source_voltage(d, duty, bat.voltage(ib)), drv.r_series, P.DT)
    ib = 4 * duty * abs(i)
    i_min = min(i_min, i)
print(f"DIR flips: {dead}   flips while PWM was non-zero: {bad}")
print(f"hard reversal peak current per motor: {i_min:.1f} A  (x4 motors = {-4*i_min:.0f} A at the motors)")
