# ArfBotOS Agent Guide

## Purpose
ArfBotOS is a mixed hardware/software robotics project centered on a 6-axis robot and vision system.
The repository combines:
- CODESYS PLC project files and scripts
- Arduino firmware for robot and remote I/O hardware
- OpenCV-based vision utilities and a small web server
- Setup, electrical, and project example documentation

## Start Here
When starting a new session in this repo, read these files first:

1. `README.md`
2. `NOTES.md`
3. `Robots/AR4/README.MD` if the work touches robot hardware, setup, or firmware
4. `Projects/HelloWorld/README.md` if the work involves first-run or demo-program behavior

## Repo Map
### `Codesys/`
- Main PLC project assets, including `ArfBot.project`, XML exports, installation config, and helper scripts.
- `Codesys/scriptengine/` is intended as a local-only copy of the CODESYS Python stub files for editor/runtime automation; do not commit it.
- To populate it locally, copy from `C:\Program Files\CODESYS 3.5.20.30\CODESYS\ScriptLib\Stubs\scriptengine`.
- CODESYS automation scripts for this repo live in `Codesys/Scripts/`; they run through the CODESYS scripting host, optionally launched from terminal via `CODESYS.exe --runscript` or a Python wrapper.
- Treat this as the core runtime/control logic area.

### `Robots/AR4/`
- AR4 robot integration docs, electrical references, setup procedures, and Arduino firmware.
- Important subareas:
- `Setup/` for tuning, mastering, and programming notes
- `Arduino/` for firmware used by the robot and remote I/O hardware
- `Electrical/` for schematics and wiring references

### `OpenCV/`
- Vision-related Python utilities, calibration helpers, template matching experiments, socket tools, and the `VisionWebServer`.
- Expect many prototype/test scripts here in addition to project-critical utilities.

### `scripts/`
- Raspberry Pi Linux installer: `scripts/install-pi.sh`.
- Intended to be run from SSH as `curl .../scripts/install-pi.sh | bash` (clones this repo if needed). Flags: `--plc-only` (no vision), `--vision-only` (no DualSense/CODESYS).
- On a PLC Pi, install the CODESYS runtime first (Tools → Update Raspberry Pi), then run this script once. It does not block if the runtime is missing; re-run afterward only to set `SysProcess=AllowAll`.
- Covers camera overlay, OpenCV, vision/controller systemd services, and CODESYS `SysProcess=AllowAll` when the runtime is already present. It does not install the CODESYS Windows IDE, runtime, licenses, or Arduino/Teensy firmware.

### `Controller/`
- Python scripts related to PlayStation DualSense controller integration.

### `Projects/`
- Example or application-specific content such as `HelloWorld`, `HelloWorld-CNC`, and `EggMaker`.

### `Resources/`
- Supporting images, device description files, and bundled reference material used by docs and setup flows.

### `scriptengine/`
- Type stubs and helper files related to CODESYS scripting support.

## Working Assumptions
- The user is usually referring to this repo root when they say "the current project".
- This is not a pure software app; changes may affect PLC behavior, robot motion, electrical integration, or vision tooling.
- Favor cautious edits and preserve existing conventions unless the user asks for a broader refactor.
- Check for documentation that already describes the workflow before changing setup-related files.

## Project-Specific Guidance
- For PLC/runtime work, inspect `Codesys/` first and use the top-level `README.md` as the architectural overview.
- For robot bring-up, use the AR4 docs before modifying firmware or setup instructions.
- For vision work, determine whether the target lives in `VisionWebServer`, a reusable utility, or a one-off test script before editing.
- For user-facing tutorials or onboarding, keep the "new automation/control engineer" audience in mind.

## Known Context
- The README describes ArfBotOS as an operating system for a 6-axis robot and vision system running on CODESYS, Arduino, and OpenCV.
- The software architecture uses command "processors" that extend a PackML-style state machine.
- The HMI is web-based and hosted through CODESYS Visu.
- The project is intended both as a working robot control stack and as an educational example of IEC-61131 structured-text industrial programming.

## Open Notes
- `NOTES.md` contains active TODO items, hardware pin notes, and handy Linux/Raspberry Pi commands.
- Treat `NOTES.md` as informal project state, not necessarily a guaranteed source of truth.

## How To Rebuild Context Fast
If a future session starts cold, do this:

1. Confirm the workspace root is `ArfBotOS`.
2. Read `README.md` for system purpose and architecture.
3. Read `NOTES.md` for current rough priorities and operational tips.
4. Read the nearest subsystem README for the area being edited.
5. Inspect the specific code or assets only after the above context is loaded.

## Maintenance
- Update this file when major architecture decisions, preferred workflows, or important project entry points change.
- Keep it short, practical, and focused on helping future sessions get productive quickly.
