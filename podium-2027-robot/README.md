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

## file structure
podium-2027-robot/
├── README.md                  # project overview, how to build, team roles
├── CONTRIBUTING.md            # branch rules, commit style, PR checklist
├── CODEOWNERS                 # who must review which folder
├── .gitignore                 # build outputs, IDE files, logs
├── .github/
│   ├── workflows/ci.yml       # build + host unit tests on every PR
│   └── pull_request_template.md
│
├── docs/
│   ├── rules/                 # rulebook PDF + our rule-notes summary
│   ├── design_report/         # final design report + extension
│   ├── interfaces.md          # shared function/CAN interface spec (key file)
│   ├── pinout.md              # pin tables for Controller A and B
│   ├── sprint_plan.md         # Sprint strategy and scoring analysis
│   └── test_logs/             # dated test results and tuning notes
│
├── hardware/
│   ├── bom/                   # drive BOM, mechanism BOMs
│   ├── wiring/                # power wiring, fuse plan, connector maps
│   └── cad_links.md           # links to the mechanical team's CAD
│
├── firmware/
│   ├── common/                # shared by both controllers
│   │   ├── can_protocol/      # message IDs, structs, pack/unpack
│   │   ├── config/            # constants: wheel size, limits, scenario code
│   │   └── utils/             # PID, filters (median/EMA), math helpers
│   │
│   ├── controller_a/          # drive, navigation, mission (Sprint lives here)
│   │   ├── Core/              # CubeMX-generated code and .ioc file
│   │   └── src/
│   │       ├── drive/         # motor PWM/DIR, braking, wheel PID
│   │       ├── sensors/       # encoders, IMU, ToF, filtering, finish detection
│   │       ├── motion/        # heading PID, straight driving, turning
│   │       ├── path/          # slalom waypoints, hurdle approach
│   │       ├── mission/       # state machine, start button, 30 s handover
│   │       ├── operator/      # iBUS receiver, manual mode, precision mode
│   │       ├── safety/        # failsafe, kill, battery, current limits
│   │       └── main.c
│   │
│   └── controller_b/          # mechanisms (later: Lift, Cycle, Marksman, Ascent)
│       ├── Core/
│       └── src/
│           ├── lift/
│           ├── cycle/
│           ├── marksman/
│           ├── ascent/
│           └── safety/
│
├── esp32_telemetry/           # tuning and telemetry link firmware
│
├── tools/
│   ├── spline_generator/      # offline path to 1 ms table (Python)
│   ├── log_plotter/           # plot encoder, IMU and PID logs
│   └── pid_tuning/            # step-response analysis scripts
│
└── tests/
    ├── unit/                  # host-side tests for PID, filters, state machine
    └── bench/                 # bench test checklists for each module

