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
python Codesys\Scripts\RunCodesysScript.py --script Codesys\Scripts\PreparePlcCommit.py --project Codesys\ArfBot.project --no-ui --text-prompts
```

## Prepare a PLC commit (stamp + export)

With the ArfBot project open:

**Tools → Scripting → Execute Script File…** → `Codesys/Scripts/PreparePlcCommit.py`

That sets `GVL_Version.sPlcVersion` to `<tag-from-main>-<n>-g<sha>[-dirty]`, saves `ArfBot.project`, and exports `Codesys/ArfBot.xml`. Then `git add` both files and commit. Download if you want the HMI to show the new string.

The tag is `git describe --tags --abbrev=0 main` (falls back to `origin/main`). The hash and commit count are from the branch you have checked out. `-dirty` is included when tracked files differ from `HEAD` (computed before the GVL write).

Stamp-only: `StampPlcVersion.py`. Export-only: `PLCOpenExport.py`.

## Cut a PLC release (clean tag, no -N-gSHA)

When this `dev` commit **is** the release, do not use `PreparePlcCommit.py` (that stamps `oldtag-N-gSHA[-dirty]`). Run `CutPlcRelease.py` instead:

**Tools → Scripting → Execute Script File…** → `Codesys/Scripts/CutPlcRelease.py`

A popup is prefilled with the next CalVer prefix (`v2.{year}.{iso-week}.{n}-`, from today's week and existing tags). Type a slug (`RoiEditor`) or edit the full tag, then confirm. That writes `sPlcVersion` to the clean tag, saves, and exports XML. It does **not** create the git tag. Commit first, then tag **that** commit from the XML stamp:

```bat
git add Codesys/ArfBot.project Codesys/ArfBot.xml
git commit -m "Stamp PLC version v2.2026.38.0-RoiEditor"
python Codesys\Scripts\TagPlcRelease.py
python Codesys\Scripts\TagPlcRelease.py --push
```

Then PR into `main` and create the GitHub Release from the existing tag (do not retag the merge commit).

Headless (no popup):

```powershell
python Codesys\Scripts\RunCodesysScript.py --script Codesys\Scripts\CutPlcRelease.py --project Codesys\ArfBot.project --output Codesys\Scripts\CutPlcRelease.out.txt --no-ui --text-prompts -- RoiEditor
```

Preview the next prefix without CODESYS:

```powershell
python Codesys\Scripts\PlcReleaseTag.py RoiEditor
```

## Included Scripts
- `RunCodesysScript.py`: Python wrapper that detects the local CODESYS install and launches a script through the CODESYS command line.
- `PreparePlcCommit.py`: Daily helper — stamps `sPlcVersion` then exports `ArfBot.xml`. Run from **Tools → Scripting → Execute Script File**.
- `CutPlcRelease.py`: Release helper — popup (or argv slug) for the next CalVer tag, stamps `sPlcVersion` to that clean tag, exports XML. Does not `git tag`.
- `TagPlcRelease.py`: After the stamp commit, create (and optionally `--push`) an annotated git tag from `sPlcVersion` in `Codesys/ArfBot.xml`.
- `PlcReleaseTag.py`: CalVer helpers (`v{major}.{year}.{iso-week}.{n}-{Slug}`). Runnable in CPython to preview the next tag.
- `StampPlcVersion.py`: Stamp-only (`GVL_Version.sPlcVersion` from git).
- `PLCOpenExport.py`: Export-only PLCopen XML next to the open `.project`.
- `parse_retain.py`: Decodes `BackupRetain.ret` / `Application.ret` program data and regenerates `st/M_BuildTests_impl.st`.
- `PatchBuildTests.py`: Writes `st/M_BuildTests_impl.st` into `_00_Main.M_BuildTests` and saves `ArfBot.project`. Does not re-export PLCopen XML (that export changes format).

One-shot probes and already-applied patches live in `archive/`.

## Temp scripts
One-shot probes, dumps, and experiments go in `Codesys/Scripts/temp/` (gitignored). Do not commit them.

Reusable visualization helpers live in the CODESYS skill (`dotfiles/.claude/skills/codesys/scripts/`). Copy into `temp/`, edit, run:

```powershell
python Codesys\Scripts\RunCodesysScript.py --script Codesys\Scripts\temp\patch_visu_rectangle.py --project Codesys\ArfBot.project --no-ui --text-prompts
```

## Notes
- CODESYS startup can be slow because each run launches the engineering environment, loads the profile, opens the project, and then runs the script.
- Prefer keeping generated outputs such as `ListDeviceTree.out.txt` out of commits unless they are intentionally needed. Logs from temp scripts should stay under `temp/`.
