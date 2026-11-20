import params as P

class PWMTimer:
    """What the STM32 timer does: limit, gate, then round duty to a whole compare value.

    BTS7960 gate stage (design report 18.1): RPWM = PWM AND DIR, LPWM = PWM AND NOT DIR.
    DIR must only change while the compare value is 0, otherwise the PWM would be
    steered from one half-bridge input to the other mid-pulse. So a reversal request
    first goes through one PWM period at compare = 0, then DIR flips.
    """
    def __init__(self):
        self.steps = P.ARR + 1
        self.dir = 1              # what the DIR pin currently holds
        self.last_compare = 0     # compare value written in the previous period

    def update(self, cmd, enable=True):
        cmd = max(-P.DUTY_CAP, min(P.DUTY_CAP, cmd))     # software duty cap
        if not enable:                                   # DRIVE_EN gate
            cmd = 0.0
        compare = round(abs(cmd) * self.steps)           # the number written to TIMx->CCRy
        want = 1 if cmd >= 0 else -1
        if compare == 0:
            pass                                         # DIR may stay where it is
        elif want != self.dir:
            if self.last_compare != 0:
                compare = 0                              # dead period: bring PWM to zero first
            else:
                self.dir = want                          # PWM is off, safe to flip DIR
        self.last_compare = compare
        return self.dir, compare / self.steps, compare
