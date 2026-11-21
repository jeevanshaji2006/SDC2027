"""
sprint_config.py  -  ASME PODIUM CY2027, The Sprint
Shared constants for the slalom planner and the hurdle planner.

FRAME (used everywhere, in Python AND in the STM32 table)
    origin  : robot start pose (centre of the robot, on the start line)
    X       : forward, along the lane            [m]
    Y       : LEFT of the lane centre-line (+)   [m]
    theta   : CCW from +X (left turn = positive) [rad]
    So "Gate 1 on the left" means Y = +0.5, "Hurdle 1 on the right" means Y = -0.5.

!! Every number marked ESTIMATE comes from reading Figure 2 (which is not to scale).
!! Change them here after the rules Q&A / after you measure the real field.
"""
import math

# ----------------------------------------------------------------- course
LANE_W      = 2.0           # rules 12.1
LANE_L      = 7.0           # start line -> finish line
FINISH_X    = LANE_L
GATE_OPENING = 0.60         # rules 12.1
HURDLE_W    = 0.60          # rules 12.1
HURDLE_H    = 0.05          # rules 12.1

# (X, Y) of the CENTRE of each opening / hurdle.      ESTIMATE (Figure 2)
GATES   = [(1.0, +0.5), (3.0, -0.5), (4.2, +0.5), (5.9, -0.5)]   # must be passed in order
HURDLES = [(2.0, -0.5), (4.8, +0.5)]

# ----------------------------------------------------------------- robot
ROBOT_W      = 0.40         # body width  (must be well under the 0.60 gate opening)
ROBOT_L      = 0.40         # body length (40 cm start cube; used for footprint checks)
WHEELBASE    = 0.30         # front axle -> rear axle          PLACEHOLDER
WHEEL_R      = 0.075        # wheel radius                      PLACEHOLDER
TRACK_EFF    = 0.45         # effective track width (skid-steer, measure by spin test)  PLACEHOLDER
MASS_KG      = 15.0         # rule 7.1-4 maximum
MU_TYRE      = 1.0          # tyre/floor friction coefficient   PLACEHOLDER
START_X      = -ROBOT_L / 2 # robot CENTRE at the start signal, relative to the start line
                            # (-0.20 = front edge on the line; use 0.0 if the centre starts on it)

# ----------------------------------------------------------------- motion limits for the planner
V_MAX        = 1.10         # m/s  cruise cap
V_HURDLE     = 0.65         # m/s  steady speed through a hurdle zone
A_ACC        = 1.20         # m/s^2
A_DEC        = 1.50         # m/s^2
A_LAT        = 1.20         # m/s^2 lateral accel cap (skid-steer scrubs hard in turns)
W_MAX        = 2.8          # rad/s yaw-rate cap

# ----------------------------------------------------------------- table / control rates
TABLE_DT     = 0.010        # s  flash table step (firmware interpolates linearly)
CONTROL_HZ   = 200          # Ramsete loop rate

# ----------------------------------------------------------------- planning margins
CORNER_CUT        = 0.03    # m  use part of the lateral tolerance to shorten the S-turns
HURDLE_MARGIN     = 0.03    # m  extra straight run beyond wheel-contact geometry
MIN_GATE_CLEAR    = 0.03    # m  minimum body-to-post clearance for a plan to be "OK"
MIN_HURDLE_MARGIN = 0.03    # m  minimum lateral margin inside the 60 cm hurdle
MAX_HURDLE_HEAD   = math.radians(5.0)
OVERRUN_M         = 0.30    # m  travel after the rear edge clears the finish line, then stop
                            #    (Q&A: is the area past the finish line still inside the Field?)

# derived: how far from the hurdle the robot CENTRE is when the wheels touch / clear it
FRONT_REACH = WHEELBASE / 2 + WHEEL_R
REAR_REACH  = WHEELBASE / 2 + WHEEL_R

# =================================================================
# BOARD A (chassis controller)  -  from "Controller A / I2C sensors and panel"
# =================================================================
BOARD_A = {
    "encoders": {   # name: (timer, chA pin, chB pin)      quadrature, hardware timer mode
        "LF": ("TIM2", "PA0", "PA1"),   # TIM2 is 32-bit
        "LR": ("TIM3", "PA6", "PA7"),   # 16-bit
        "RF": ("TIM4", "PB6", "PB7"),   # 16-bit
        "RR": ("TIM8", "PC6", "PC7"),   # 16-bit
    },
    "pwm": {        # name: (timer channel, PWM pin, DIR pin)
        "LF": ("TIM1_CH1", "PA8",  "PB0"),
        "LR": ("TIM1_CH2", "PA9",  "PB1"),
        "RF": ("TIM1_CH3", "PA10", "PB2"),
        "RR": ("TIM1_CH4", "PA11", "PC2"),
    },
    "i2c": {"scl": "PB8", "sda": "PB9",
            "devices": {"BMI088_accel": 0x18, "BMI088_gyro": 0x68,
                        "VL53L1X_left": 0x2A, "VL53L1X_right": 0x2B,
                        "MCP23017_panel": 0x20}},
    "tof_xshut": {"left": "PC8", "right": "PC9"},
    "drive_en": "PC3",          # hardware permission for the 4 PWM gates
    "mech_en": "PC1",           # to board B (not used by the Sprint)
    "start": "PD2",             # START contact to GND (active low)
    "ibus_rx": "PB11",          # FlySky FS-iA6B receiver, USART3
    "vbat_adc": "PA4",
    "can": ("PB3", "PB4"),      # TCAN332 board link (not used by the Sprint)
    "panel": {"sel_sprint": "GPA0", "ready_led": "GPB0", "auto_led": "GPB1", "fault_led": "GPB2"},
}


def check_board_a(verbose=True):
    """Sanity-check the Board A table against the diagram (27 MCU pins, unique I2C addresses)."""
    b = BOARD_A
    pins = []
    for t, a, c in b["encoders"].values():
        pins += [a, c]
    for _, p, d in b["pwm"].values():
        pins += [p, d]
    pins += [b["i2c"]["scl"], b["i2c"]["sda"], *b["tof_xshut"].values(),
             b["drive_en"], b["mech_en"], b["start"], b["ibus_rx"], b["vbat_adc"], *b["can"]]
    assert len(pins) == len(set(pins)), "duplicate MCU pin in Board A map"
    addrs = list(b["i2c"]["devices"].values())
    assert len(addrs) == len(set(addrs)), "duplicate I2C address"
    assert len(pins) == 27, f"expected 27 MCU pins from the diagram, got {len(pins)}"
    if verbose:
        print(f"[hw] Board A map OK: {len(pins)} MCU pins, {len(addrs)} I2C devices on one bus")


def i2c_budget(scl_hz, gyro_hz=400, accel_hz=200, tof_hz=50, n_tof=2):
    """Bus utilisation of the shared PB8/PB9 bus (blocking-read estimate)."""
    def xfer_bits(data_bytes):             # addr+reg+addr(restart)+data, 9 bits/byte, +start/stop
        return (3 + data_bytes) * 9 + 4
    load = (gyro_hz * xfer_bits(6) +       # BMI088 gyro x/y/z
            accel_hz * xfer_bits(7) +      # BMI088 accel (1 dummy byte + 6)
            n_tof * tof_hz * xfer_bits(17)) / scl_hz   # VL53L1X result block, per sensor
    return load


if __name__ == "__main__":
    check_board_a()
    for f in (100_000, 400_000):
        print(f"[hw] I2C {f//1000:>3} kHz: IMU 400/200 Hz + 2x ToF 50 Hz -> {100*i2c_budget(f):5.1f} % bus load")