# Contributing

## Branches
- `main` is always working code. **No direct pushes.**
- Branch names: `feature/<name>-<topic>`, `fix/<name>-<topic>`, `test/<name>-<topic>`
  (example: `feature/aksh-wheel-pid`).

## Commits
Short, present tense, one idea each: `drive: add per-wheel PID`, `path: tune slalom waypoints`.
Prefix with the module folder name.

## Pull requests
1. Keep PRs small (one module, one purpose).
2. Fill in the PR template.
3. Code must build. Logic-only code (PID, filters, state machine) should have a host unit test.
4. The software head reviews and merges. Owners in `CODEOWNERS` must approve changes in their folder.

## Rules
- Only edit files in your own module. If you need an interface change, edit
  `docs/interfaces.md` in a separate PR and tag the software head.
- Never change `firmware/common/` without the software head.
- No hardcoded numbers: put constants in `firmware/common/config/`.
- Units are fixed (see `docs/interfaces.md`). Always put the unit in the name (`speed_mps`, `heading_deg`).
- Do not commit logs, videos or CAD. Link them instead.
- Tag tested versions: `sprint-v0.1`, `sprint-v0.2` ...
