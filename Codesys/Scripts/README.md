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

## Notes
- CODESYS startup can be slow because each run launches the engineering environment, loads the profile, opens the project, and then runs the script.
- Prefer keeping generated outputs such as `ListDeviceTree.out.txt` out of commits unless they are intentionally needed.
