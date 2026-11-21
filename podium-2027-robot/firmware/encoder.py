import math
import params as P

class Encoder:
    """Quadrature encoder read by an STM32 timer in encoder mode (x4 decoding).

    The timer only holds a free-running COUNTER REGISTER (16-bit on TIM3/4/8, 32-bit on TIM2).
    It wraps. Firmware never uses the absolute value, only the difference between two reads.
    """
    def __init__(self, cpr=P.ENC_CPR, bits=P.ENC_BITS, start_count=0, at_motor_shaft=True):
        self.cpr, self.bits, self.mask = cpr, bits, (1 << bits) - 1
        self.k = 1.0 if at_motor_shaft else 1.0 / P.N      # shaft the encoder sits on, relative to the motor shaft
        self.theta = 0.0                                   # true encoder-shaft angle, rad
        self.offset = start_count                          # lets a test start the counter near the wrap point

    def update(self, w_motor, dt):
        self.theta += w_motor * self.k * dt

    @property
    def register(self):
        """What reading TIMx->CNT would return right now."""
        return (self.offset + math.floor(self.theta * self.cpr / (2 * math.pi))) & self.mask


class SpeedEstimator:
    """Motor-shaft speed from encoder register reads, once per control tick (Ts = 1/CTRL_HZ).

    window = number of ticks the speed is averaged over (1 = raw delta of the last tick).
    One count is worth  2*pi/cpr/(window*Ts)  rad/s, so a longer window = finer resolution, more lag.
    """
    def __init__(self, enc, window=1, ts=1.0 / P.CTRL_HZ):
        self.enc, self.window, self.ts = enc, window, ts
        self.hist = [enc.register] * (window + 1)           # register values at the last window+1 ticks
        self.w = 0.0

    def tick(self):
        self.hist = self.hist[1:] + [self.enc.register]
        delta = (self.hist[-1] - self.hist[0]) & self.enc.mask          # modular subtraction handles the wrap
        if delta > self.enc.mask >> 1:                                   # reinterpret as signed
            delta -= self.enc.mask + 1
        self.w = delta * (2 * math.pi / self.enc.cpr) / (self.window * self.ts) / self.enc.k
        return self.w                                                    # motor-shaft rad/s

    @property
    def output_rpm(self):
        return self.w / P.N * 60 / (2 * math.pi)
