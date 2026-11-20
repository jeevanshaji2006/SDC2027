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
