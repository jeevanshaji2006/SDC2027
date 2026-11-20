# Interfaces (v0.1 DRAFT)

**Purpose:** everyone codes against these functions so modules fit together without waiting for each other.
Changing anything here needs a separate PR approved by the software head.

## Conventions
- Language: C (STM32 HAL). Each module has `module.h` / `module.c`.
- **Units:** distance in **m**, speed in **m/s**, angular rate in **rad/s**, heading in **deg**
  (0 = lane direction, positive = counter-clockwise), time in **ms**, voltage in **V**, current in **A**.
- Put the unit in the name: `speed_mps`, `heading_deg`, `range_mm`.
- Return `bool` (true = ok) or a status enum. No blocking waits longer than 1 ms in control loops.
- Coordinates: origin at the start line centre, **x** along the lane (toward the finish), **y** to the left.

## Loop rates (Controller A)
| Loop | Rate | Contains |
|---|---|---|
| Control | 1 kHz | wheel PID, heading PID, safety checks |
| Estimation | 200 Hz | pose, filtered sensors |
| Mission | 100 Hz | state machine, path following, operator input |

## 1. Drive (owner: Aksh) `drive.h`
```c
void  drive_init(void);
void  drive_set_wheel_speed(wheel_t w, float speed_mps);   /* closed loop, per wheel */
void  drive_set_velocity(float v_mps, float omega_radps);  /* body command, mixes to wheels */
void  drive_brake(void);                                   /* active brake, keeps enable on */
void  drive_stop(void);                                    /* coast, zero command */
void  drive_enable(bool on);                               /* software request for DRIVE_EN */
bool  drive_is_stalled(wheel_t w);                         /* from current sense */
```

## 2. Motion (owner: Aksh) `motion.h`
```c
void  motion_hold_heading(float heading_deg, float v_mps); /* drive straight on a heading */
bool  motion_turn_to(float heading_deg);                   /* true when reached within tolerance */
bool  motion_drive_distance(float dist_m, float v_mps);    /* true when reached */
void  motion_cancel(void);
```

## 3. Sensors (owner: Joshua) `sensors.h`
```c
typedef struct { float x_m, y_m, heading_deg, speed_mps; } pose_t;

void    sensors_init(void);
void    sensors_update(void);              /* call at 200 Hz, filters inside */
pose_t  sensors_get_pose(void);
float   sensors_get_heading_deg(void);
float   sensors_get_pitch_deg(void);       /* for hurdle detection */
float   sensors_get_distance_m(void);      /* odometry along the lane */
bool    sensors_get_range_mm(range_id_t id, uint16_t *mm); /* false if status invalid */
bool    sensors_finish_detected(void);     /* finish line reached, see docs/sprint_plan.md */
```
Filtering: 7-sample median on ToF, complementary or EKF fusion for pose.

## 4. Path (owner: Nile) `path.h`
```c
void  path_init(void);
void  path_reset(void);
void  path_update(pose_t pose, float *v_cmd_mps, float *omega_cmd_radps); /* 100 Hz */
int   path_gate_index(void);   /* 0..4, number of Gates passed so far */
bool  path_gate1_passed(void); /* used for the +10 Autonomous bonus */
bool  path_done(void);
```
Hurdle handling is inside the path module: approach straight, steady speed, pitch-triggered boost.

## 5. Mission (owner: Joshua) `mission.h`
```c
typedef enum { M_IDLE, M_ARMED, M_AUTO, M_CREWED, M_FINISHED, M_SAFE } mission_state_t;

void             mission_init(void);
void             mission_update(void);      /* 100 Hz */
mission_state_t  mission_state(void);
uint32_t         mission_time_ms(void);
```
Flow: `IDLE -> ARMED (start button, short delay for Crew to step away) -> AUTO (30 s)
-> CREWED (2 min, manual control) -> FINISHED`. Any fault goes to `M_SAFE`.
If the Robot has not finished when AUTO ends, hand over to manual control without stopping.

## 6. Operator (owner: Software head) `operator.h`
```c
typedef struct { float throttle, steer; bool precision, kill, arm; bool link_ok; } operator_t;

void        operator_init(void);
operator_t  operator_read(void);            /* iBUS, filtered, deadband applied */
void        operator_to_command(operator_t in, float *v_mps, float *omega_radps);
```
Precision mode scales speed to 30 percent. Smooth driving: slew-rate limit plus an expo curve.

## 7. Safety (owner: Software head) `safety.h`
```c
void   safety_init(void);
void   safety_update(void);                  /* 1 kHz */
bool   safety_ok(void);
fault_t safety_faults(void);                 /* bit flags */
void   safety_trigger(fault_t f);            /* latches and disables drive */
```
Fault triggers: link loss (no valid iBUS frame for 100 ms), kill command, low battery
(under 13.2 V loaded on 4S for 200 ms), motor overcurrent, tilt above 35 deg, watchdog.
Response: `drive_stop()`, disable enable line, latch until re-armed.

## 8. Shared constants
`firmware/common/config/robot_config.h`. Never duplicate these numbers in module code.

## 9. CAN (A to B), for later
Message IDs and frames follow Extension Section 19. Sprint does not need Controller B.
Heartbeat `HB_A` 0x010 at 20 ms is still sent by A.

## Change log
| Version | Date | Change |
|---|---|---|
| 0.1 | 2026-11-20 | First draft |
