import math
import params as P

class Motor:
    """Brushed DC motor + gearbox. State: motor-shaft speed w (rad/s)."""
    def __init__(self):
        self.w = 0.0

    def step(self, v_source, r_extra, dt):
        # electrical side (settles in ~1 ms, so solved directly): v = ke*w + (R + R_driver)*i
        i = (v_source - P.KE * self.w) / (P.R_MOTOR + r_extra)
        torque = P.KE * i
        fric = P.KE * P.I0
        # mechanical side: J*dw/dt = torque - friction
        if abs(self.w) < 1e-3 and abs(torque) <= fric:
            self.w = 0.0                                  # stuck by static friction
        else:
            sign = self.w if self.w != 0 else torque
            self.w += dt * (torque - math.copysign(fric, sign)) / P.J
        return i

    @property
    def output_rpm(self):
        return self.w / P.N * 60 / (2 * math.pi)
