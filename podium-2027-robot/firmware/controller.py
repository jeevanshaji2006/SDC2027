import math
import params as P

def ms_to_motor_w(v):
    """wheel speed in m/s -> motor-shaft rad/s (through wheel radius and gearbox)"""
    return v / P.R_WHEEL * P.N

def motor_w_to_ms(w):
    return w / P.N * P.R_WHEEL

class SpeedLoop:
    """One wheel's speed controller: rate limit -> feedforward -> PI -> duty.
    Works in motor-shaft rad/s. Call update() once per control tick (1 kHz)."""
    def __init__(self, kS, kV, kP, kI, accel_ms2, ts=1.0 / P.CTRL_HZ):
        self.kS, self.kV, self.kP, self.kI = kS, kV, kP, kI
        self.max_step = ms_to_motor_w(accel_ms2) * ts      # biggest allowed change of the reference per tick
        self.ts = ts
        self.ref = 0.0       # the ramped target the loop actually chases
        self.integ = 0.0     # PI integral, in volts

    def update(self, w_target, w_meas, v_bus):
        # 1. acceleration limit: the reference moves toward the target by at most max_step per tick
        self.ref += max(-self.max_step, min(self.max_step, w_target - self.ref))

        # 2. feedforward: the voltage the motor model says this speed needs
        ff = (self.kS * math.copysign(1, self.ref) + self.kV * self.ref) if self.ref != 0 else 0.0

        # 3. PI on the measured speed error
        err = self.ref - w_meas
        volts = ff + self.kP * err + self.integ

        # 4. volts -> duty, using the measured battery voltage, then the duty cap
        duty_raw = volts / v_bus
        duty = max(-P.DUTY_CAP, min(P.DUTY_CAP, duty_raw))

        # 5. anti-windup: do not keep integrating while saturated and still pushing the same way
        if not (abs(duty_raw) > P.DUTY_CAP and err * duty_raw > 0):
            self.integ += self.kI * err * self.ts
        if self.ref == 0 and abs(w_meas) < 5.0:
            self.integ = 0.0                                # parked: forget old integral
        return duty
