"""All numbers in one place. (ASSUMED) = a guess to be replaced by a measurement."""
import math

# ---- Bus choice (design report 2.3): 1 = 4S + 12 V motors, 2 = 6S + 24 V motors ----
BUS_OPTION = 2

# ---- Battery, modelled as a source V_OC in series with R_BATT ----
if BUS_OPTION == 1:
    V_OC, R_BATT, V_NOM = 16.8, 0.06, 12.0   # 4S 3000 mAh, 12 V motors. R_BATT ASSUMED
else:
    V_OC, R_BATT, V_NOM = 25.2, 0.05, 24.0   # 6S, 24 V motors. R_BATT ASSUMED (measure the real pack)

# ---- Driver: BTS7960 ----
R_DRV    = 0.03    # ohm series drop (BOM: 16 mohm per path, plus wiring/connectors; ASSUMED)
DUTY_CAP = round(V_NOM / V_OC, 3)  # motor rated voltage / full-pack voltage: 0.714 (4S) or 0.952 (6S)

# ---- STM32 timer (TIM1): PWM frequency and resolution ----
TIM_CLK  = 170e6                       # Hz, G474 timer clock at 170 MHz
PWM_FREQ = 20e3                        # Hz
ARR      = round(TIM_CLK / PWM_FREQ) - 1   # auto-reload value -> 8499, i.e. 8500 steps

# ---- Motor: PG36M555, 19.2:1 (numbers from the BOM) ----
RPM_BASE, N = 6000.0, 19.2
I_SCALE = 12.0 / V_NOM       # 1 for the 12 V BOM motor. ASSUMED for 24 V: same frame, winding for 2x voltage
                             # -> same speed and torque, half the current, 4x the winding resistance.
                             # Replace with the real 24 V datasheet numbers when known.
RPM_RATED, T_RATED, I_RATED = 262.0, 0.45, 1.95 * I_SCALE   # T in N*m (45 N*cm)
I0 = 0.3 * I_SCALE                                          # A, no-load current (ASSUMED)

KE      = V_NOM / (RPM_BASE * 2 * math.pi / 60)                       # V*s/rad (= kt)
R_MOTOR = (V_NOM - KE * RPM_RATED * N * 2 * math.pi / 60) / I_RATED   # ohm
ETA     = T_RATED / (N * KE * (I_RATED - I0))                         # gearbox efficiency

# ---- Inertia seen at the motor shaft ----
M_ROBOT, R_WHEEL = 15.0, 0.075    # kg, m (150 mm wheels, design report 2.1)
J_ROTOR = 1.0e-5                  # kg*m^2 (ASSUMED)
J = J_ROTOR + (M_ROBOT * R_WHEEL**2 / 4) / N**2   # rotor + this wheel's quarter of the robot

# ---- Wheel encoders (NOT in the drive BOM yet: every number here is ASSUMED) ----
ENC_PPR   = 11          # pulses per rev per channel, on the MOTOR shaft (typical 36 mm gearmotor Hall encoder)
ENC_CPR   = 4 * ENC_PPR # counts per motor rev, x4 quadrature decoding (TIMx encoder mode)
ENC_BITS  = 16          # TIM3/TIM4/TIM8 are 16-bit; TIM2 (LF) is 32-bit
CTRL_HZ   = 1000        # velocity-loop tick (report section 23: 1 kHz)

# ---- Simulation step ----
DT = 50e-6   # s
