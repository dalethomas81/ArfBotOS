# CODESYS Scripts

This folder contains Python scripts intended to run inside the CODESYS scripting host.

## Requirements
- CODESYS must be installed locally.
- These scripts are launched through `CODESYS.exe`, either manually or by using `RunCodesysScript.py`.
- The local `Codesys/scriptengine/` folder is for editor/type-stub support only and is not committed to git.

## Local Stub Setup
If you want local code completion for the CODESYS scripting API, copy the stub package from:

`C:\Program Files\CODESYS 3.5.20.30\CODESYS\ScriptLib\Stubs\scriptengine`

into:

`Codesys/scriptengine`

This folder is ignored by git and should stay local-only.

## Running Scripts
Example:

```powershell
python Codesys\Scripts\RunCodesysScript.py --script Codesys\Scripts\ListDeviceTree.py --project Codesys\ArfBot.project --no-ui --text-prompts
```

## Included Scripts
- `RunCodesysScript.py`: Python wrapper that detects the local CODESYS install and launches a script through the CODESYS command line.
- `ListDeviceTree.py`: Opens the target project if needed and writes the CODESYS device tree to `Codesys/Scripts/ListDeviceTree.out.txt`.
- `PatchTuningDeadTime.py`: Writes the deadtime-suggestion logic into `_00_Main` / `_M_Tuning` and relabels the Tuning DeadTime field. ST sources live in `Codesys/Scripts/st/`.
- `parse_retain.py`: Decodes `BackupRetain.ret` / `Application.ret` program data and regenerates `st/M_BuildTests_impl.st`.
- `PatchBuildTests.py`: Writes `st/M_BuildTests_impl.st` into `_00_Main.M_BuildTests` and saves `ArfBot.project`. Does not re-export PLCopen XML (that export changes format).
- `PatchLicenseStatus.py`: Creates/updates `FB_LicenseStatus`, wires `GVL.LicenseStatus` and `_00_Main`, adds Component Manager + CmpEventMgr, then builds.

## Temp scripts
One-shot probes, dumps, and experiments go in `Codesys/Scripts/temp/` (gitignored). Do not commit them.

Reusable visualization helpers live in the CODESYS skill (`dotfiles/.claude/skills/codesys/scripts/`). Copy into `temp/`, edit, run:

```powershell
python Codesys\Scripts\RunCodesysScript.py --script Codesys\Scripts\temp\patch_visu_rectangle.py --project Codesys\ArfBot.project --no-ui --text-prompts
```

## Notes
- CODESYS startup can be slow because each run launches the engineering environment, loads the profile, opens the project, and then runs the script.
- Prefer keeping generated outputs such as `ListDeviceTree.out.txt` out of commits unless they are intentionally needed. Logs from temp scripts should stay under `temp/`.
