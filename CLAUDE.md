# CLAUDE.md — NxGV Carbot Autonomous V2

This file is instructions for Claude Code working inside this repo. Read it
first, every session, before touching any file.

## What this repo is

ROS 2 Humble / Ubuntu 22.04 stack for the RISA Bot, NxGV Driverless CarBot
Challenge 26/27 (25–26 Sept 2026, MARII Cyberjaya). Built in phases, one
phase per work session, across many separate chats/sessions with the owner.

Target repo (this one): https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2
Base repo (drivers/launch/motor-servo/GUI to extend, not rewrite):
https://github.com/Aquadrox-Technologies/NXGV-Driveless-Carbot-Challenge

## Read these before doing anything

Everything below already lives in the repo. Do not ask the owner to re-paste
it, and do not re-derive it from memory — read the files.

1. `docs/PHASES.md` — the phase table (done/next), and a "what exists" /
   "handoff to next phase" section per completed phase. This is the source
   of truth for repo state. Trust it, but spot-check against actual code
   before relying on a "done" mark (see Verification below).
2. `docs/BACKLOG.md` — open items, known gaps, anything marked
   untested-on-hardware.
3. `docs/PROJECT_BRIEF.md` (if present) — the full original spec: rules,
   architecture blocks, camera setup, UWB handoff, GUI tabs, calibration
   wizard order, race-mode flow. If it isn't there yet, ask the owner
   whether to add it from the chat history, but don't block work on it —
   `docs/PHASES.md` + `docs/BACKLOG.md` are enough to resume a phase.
4. `docs/reference/` — rulebook PDF, architecture visual guide, V4
   simulator JS sources, Camera_Setup, UWB_Handoff, firmware headers, spec
   sheet, GUI screenshots. These are the algorithmic/spec ground truth —
   port faithfully, don't improvise parameters.
5. `carbot_common/topics.py`, `carbot_interfaces/msg`, `config/params/*.yaml`,
   `config/data/*.yaml` — the actual contract. Read the current values here,
   not from a summary.

## Hard rules (do not violate, ever)

- Never rename an existing topic, message type, or YAML key silently. If a
  change is genuinely needed, stop and say so explicitly before doing it.
- Every tunable lives in YAML. A missing key is a loud error, never a
  silent default.
- UWB never reaches the servo, and never makes the local estimate jump. Two
  separate pose estimates: local (smooth, steers) and coarse/global (UWB
  weighted in, used only for route/checkpoint/roundabout identity).
- Roundabout exits are chosen by the global planner from `mission.yaml`
  checkpoints — never by the boom-gate detector. Gate/route disagreement is
  logged and shown in the GUI, never changes the route.
- No timer-based traffic-light logic. Pre-recorded/replayed motion is
  allowed ONLY where the owner has confirmed it (check `docs/BACKLOG.md` /
  `PHASES.md` for the current parking strategy — this has changed before;
  don't assume, grep for it).
- One command-owner node is the only writer of the final motion command
  topic; it has a safety veto.
- Never commit secrets. The real `TagConfig.h` (WiFi creds, agent IP) must
  stay out of git — only `TagConfig.example.h` with placeholders is
  committed. Check `.gitignore` and `tools/git-hooks/pre-commit` are intact
  before any commit that touches firmware config.

## Working process

1. At the start of a session, clone or pull the repo fresh and read
   `docs/PHASES.md` to find the current phase and its handoff notes.
2. Before writing code for a phase, list what's still needed from the owner
   (missing reference file, an ambiguous rule, hardware you can't verify
   from here) rather than guessing.
3. Build only the current phase. Don't reach ahead into later phases'
   files unless a stub already exists for them from Phase 1.
4. Write/run tests for what you build (pure-Python packages have `test/`
   dirs runnable without ROS via pytest; ROS-only checks are marked
   skip-outside-ROS and get verified on the RDK separately).
5. Update `docs/PHASES.md` (mark the phase done, add its "what exists" and
   "handoff to next phase" notes) and `docs/BACKLOG.md` (open items) as
   part of the same commit — not as an afterthought.
6. Commit with a clear phase-scoped message, push to `main`.
7. End every session by stating: files changed, the git commands used, how
   to test this phase on the actual car (RDK X5), and anything the next
   phase/session needs to know.

## Verification (don't trust a "done" mark blindly)

Before treating a phase as complete, actually check the live repo, not
just the doc:
```bash
git clone --depth 50 https://github.com/reCONNECT02/NxGV-Carbot-Autonomous-V2 /tmp/check
cd /tmp/check && git log --oneline -20
```
Run the pure-Python test suites per package (`pytest`), and use
`tools/sandbox/` runners where they exist to sanity-check logic without
ROS. ROS-node-level behavior only gets confirmed on the RDK X5 itself —
say clearly when something is still untested on hardware.

## Environment notes

- The owner drives this mostly from **Windows PowerShell**, sometimes SSH'd
  into the RDK X5 (Ubuntu 22.04, ARM, RDK X5 8GB). PowerShell has no `&&`
  chaining and no `unzip` — give separate lines, or `Expand-Archive` for
  zips, when instructing them directly.
- `mipi_cam` must run as root — any launch/systemd instructions must handle
  that explicitly.
- Camera roles (front/left/right) are confirmed in YAML + the calibration
  wizard, never hardcoded.

## Current phase status

Don't hardcode a status here — it goes stale. Always read
`docs/PHASES.md` for the live table.
