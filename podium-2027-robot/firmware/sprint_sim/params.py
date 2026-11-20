"""All numbers in one place. (ASSUMED) = a guess to be replaced by a measurement."""
import math

# ---- Battery: 4S LiPo, modelled as a source V_OC in series with R_BATT ----
V_OC   = 16.8      # V, fully charged 4S
R_BATT = 0.06      # ohm  (ASSUMED, measure later)

# ---- Driver: BTS7960 ----
R_DRV    = 0.03    # ohm series drop (BOM: 16 mohm per path, plus wiring/connectors; ASSUMED)
DUTY_CAP = round(12.0 / 16.8, 3)   # = 0.714: 12 V motor on a full 16.8 V pack. (BOM said 0.75 -> 12.6 V; report 20.4 says ~71%)

# ---- STM32 timer (TIM1): PWM frequency and resolution ----
TIM_CLK  = 170e6                       # Hz, G474 timer clock at 170 MHz
PWM_FREQ = 20e3                        # Hz
ARR      = round(TIM_CLK / PWM_FREQ) - 1   # auto-reload value -> 8499, i.e. 8500 steps

# ---- Motor: PG36M555, 19.2:1 (numbers from the BOM) ----
V_NOM, RPM_BASE, N = 12.0, 6000.0, 19.2
RPM_RATED, T_RATED, I_RATED = 262.0, 0.45, 1.95   # T in N*m (45 N*cm)
I0 = 0.3                                          # A, no-load current (ASSUMED)

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
