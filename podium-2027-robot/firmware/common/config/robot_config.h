#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Units: distance m, speed m/s, angle rad unless the name says otherwise. */

/* ---- Robot ---- */
#define WHEEL_DIAMETER_M        0.150f
#define TRACK_WIDTH_M           0.320f   /* TODO: measure on the real robot */
#define ENCODER_CPR             0        /* TODO: counts per wheel rev after gearbox */
#define MAX_SPEED_MPS           2.0f     /* ~2.06 m/s at rated load; cap lower in cruise */

/* ---- Sprint field (Rulebook 12.1) ---- */
#define SPRINT_LANE_LENGTH_M    7.0f
#define SPRINT_LANE_WIDTH_M     2.0f
#define SPRINT_GATE_COUNT       4
#define SPRINT_GATE_WIDTH_M     0.60f
#define SPRINT_GATE_SPACING_M   1.4f
#define SPRINT_HURDLE_COUNT     2
#define SPRINT_HURDLE_HEIGHT_M  0.05f

/* ---- Timing (Rulebook 11.3) ---- */
#define AUTO_PHASE_S            30
#define CREWED_PHASE_S          120

/* ---- Control loop rates ---- */
#define CONTROL_LOOP_HZ         1000
#define MISSION_LOOP_HZ         100
#define CAN_HEARTBEAT_MS        20

#endif
