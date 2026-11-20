# PODIUM CY2027 Robot

Software, firmware, hardware docs and tools for our ASME Student Design Competition
PODIUM (CY2027) Robot. **Current focus: The Sprint** (100 points, drive only).

> Keep this repository **private** until the competition. It contains our strategy.

## Robot at a glance

| Item | Value |
|---|---|
| Controllers | 2x NUCLEO-G474RE (A = drive/nav/mission, B = mechanisms) |
| Drive | 4WD skid steer, 4x PG36M555, 150 mm wheels, BTS7960 drivers |
| Limits | 15 kg, 24 V max, 40 cm start cube, 75 cm wide / 120 cm tall deployed |
| Run | 30 s Autonomous Phase + 2 min Crewed Phase |

## Sprint in one paragraph

2.0 m x 7.0 m lane, 4 Gates (60 cm, alternating sides, ~1.4 m apart), 2 Hurdles (5 cm).
Score = 100 - 1 per second - 15 per missed Gate - 10 per Hurdle gone around - 15 per Reset,
+10 if Gate 1 is passed in the Autonomous Phase. Capped at 0-100. **Reliability beats speed.**
Details: `docs/sprint_plan.md`.

## Repository layout

| Path | Contents |
|---|---|
| `docs/` | Rules, design report, interface spec, pinout, Sprint plan, test logs |
| `hardware/` | BOMs, wiring and fuse plan |
| `firmware/common/` | Code shared by both controllers (CAN protocol, config, PID/filters) |
| `firmware/controller_a/` | Drive, sensors, motion, path, mission, operator, safety |
| `firmware/controller_b/` | Lift, Cycle, Marksman, Ascent (later) |
| `esp32_telemetry/` | Wireless tuning and telemetry |
| `tools/` | Spline generator, log plotter, PID tuning scripts |
| `tests/` | Host unit tests and bench test checklists |

## Module owners (Sprint)

| Module | Folder | Owner |
|---|---|---|
| Drive and motion (PWM/DIR, wheel PID, heading PID) | `controller_a/src/drive`, `motion` | Aksh |
| Sensors, filtering, finish detection, mission state machine | `controller_a/src/sensors`, `mission` | Joshua |
| Slalom path, hurdle approach, Gate 1 autonomous bonus | `controller_a/src/path` | Nile |
| Operator, safety, interfaces, integration, tuning | `controller_a/src/operator`, `safety`, `docs/interfaces.md` | Software head |

## Getting started

1. Clone: `git clone <repo-url>`
2. Install STM32CubeIDE (or CubeMX + arm-none-eabi-gcc) and open `firmware/controller_a/`.
3. Read `docs/interfaces.md` **before writing any code**.
4. Create a branch, work, open a pull request. See `CONTRIBUTING.md`.

## Rules reminders that affect code

- No practice Runs; every Run counts. One Run per Discipline.
- Autonomous Phase: no human input after the single start press.
- Robot must be disable-able on request (main switch or control link).
- One control link only (tether must stay slack, or one wireless link).
- Leaving the Field voids the Run, so stop just past the finish line.
