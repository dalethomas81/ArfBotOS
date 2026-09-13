# Stamp GVL_Version.sPlcVersion from git, then export Codesys/ArfBot.xml.
#
# Daily commit helper: Tools -> Scripting -> Execute Script File
# Or: python Codesys\Scripts\RunCodesysScript.py --script Codesys\Scripts\PreparePlcCommit.py --project Codesys\ArfBot.project --no-ui --text-prompts
from __future__ import print_function
import os
import sys
import traceback

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)
import StampPlcVersion as stamp

if not (len(sys.argv) > 2 and sys.argv[2]):
    stamp.LOG_FILE = os.path.join(_SCRIPT_DIR, "PreparePlcCommit.out.txt")


def export_plcopen_xml(proj):
    filename = str(os.path.splitext(proj.path)[0] + ".xml")
    stamp.emit("Exporting PLCopen XML: {0}".format(filename))
    objects = proj.get_children()
    proj.export_xml(
        objects,
        path=filename,
        recursive=True,
        export_folder_structure=True,
        declarations_as_plaintext=True,
    )
    stamp.emit("XML export done")


def main():
    stamp.emit("PreparePlcCommit starting")
    proj = stamp.ensure_project()
    stamp.emit("Project: {0}".format(proj.path))

    git_root = stamp.find_git_root(proj.path)
    stamp.emit("Git root: {0}".format(git_root))

    version = stamp.describe_version(git_root)
    if len(version) > stamp.MAX_VERSION_LEN:
        raise RuntimeError(
            "Version '{0}' is {1} chars; STRING({2}) max".format(
                version, len(version), stamp.MAX_VERSION_LEN
            )
        )
    stamp.emit("sPlcVersion := '{0}'".format(version))

    gvl = stamp.find_textual(proj, "GVL_Version")
    if gvl is None or not gvl.has_textual_declaration:
        raise RuntimeError("GVL_Version not found or has no textual declaration")

    old = gvl.textual_declaration.text
    new = stamp.stamp_declaration(old, version)
    if new == old:
        stamp.emit("GVL_Version.sPlcVersion already set")
    else:
        gvl.textual_declaration.replace(new_text=new)
        stamp.emit("Updated GVL_Version.sPlcVersion")

    proj.save()
    stamp.emit("Project saved")
    export_plcopen_xml(proj)
    stamp.emit("script finished.")


try:
    main()
except Exception as exc:
    stamp.emit("ERROR: {0}".format(exc))
    stamp.emit(traceback.format_exc())
    stamp.write_log()
    if not stamp.ALREADY_OPEN:
        try:
            system.exit(1)
        except Exception:
            pass
else:
    stamp.write_log()
    if not stamp.ALREADY_OPEN:
        try:
            system.exit(0)
        except Exception:
            pass
