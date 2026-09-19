# Stamp GVL_Version.sPlcVersion with the next release tag, then export XML.
#
# Daily WIP still uses PreparePlcCommit.py (oldtag-N-gSHA[-dirty]).
# Run this when the current dev commit is the release:
#   Tools -> Scripting -> Execute Script File -> CutPlcRelease.py
# A popup asks for the slug (or a full tag). The script does not git tag.
#
# Headless (no popup): pass the slug after --
#   python Codesys\Scripts\RunCodesysScript.py --script Codesys\Scripts\CutPlcRelease.py --project Codesys\ArfBot.project --no-ui --text-prompts -- RoiEditor
from __future__ import print_function
import os
import sys
import traceback

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)
import StampPlcVersion as stamp
import PlcReleaseTag as reltag

stamp.bind_host(sys.modules.get("__main__"))
if not (len(sys.argv) > 2 and sys.argv[2]):
    stamp.LOG_FILE = os.path.join(_SCRIPT_DIR, "CutPlcRelease.out.txt")


def extra_script_args():
    if stamp.launched_headless():
        start = 3
    else:
        start = 1
    result = []
    for raw in sys.argv[start:]:
        value = stamp.clean_arg(raw)
        if value and value != "--":
            result.append(value)
    return result


def parse_flags(args):
    slug = None
    dry_run = False
    skip_confirm = False
    for arg in args:
        lower = arg.lower()
        if lower in ("--dry-run", "dry-run", "-n"):
            dry_run = True
            continue
        if lower in ("--yes", "-y", "yes"):
            skip_confirm = True
            continue
        if slug is None:
            slug = arg
    return slug, dry_run, skip_confirm


def ui_present():
    try:
        return bool(system.ui_present)
    except Exception:
        return False


def list_tags(git_root):
    listing, _code = stamp.run_git(["tag", "--list"], git_root)
    return [line.strip() for line in listing.splitlines() if line.strip()]


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


def confirm_stamp(tag):
    try:
        choice = PromptChoice.YesNo
        default = PromptResult.Yes
        yes = PromptResult.Yes
    except NameError:
        stamp.emit("Prompt enums missing; skipping confirm dialog")
        return True
    result = system.ui.prompt(
        "Write sPlcVersion := '{0}' and export ArfBot.xml?\n"
        "This does not create the git tag. Commit first, then tag that commit.".format(tag),
        choice,
        default,
    )
    return result == yes


def ask_tag(prefix):
    message = (
        "Release tag for GVL_Version.sPlcVersion.\n"
        "Prefix {0} is today's ISO week (next unused n).\n"
        "Append a feature slug after the hyphen, or edit the full tag. Cancel aborts.".format(prefix)
    )
    entered = system.ui.query_string(
        message,
        text=prefix,
        multi_line=False,
        cancellable=True,
    )
    if entered is None:
        raise RuntimeError("CutPlcRelease cancelled")
    return entered


def follow_up_commands(tag):
    stamp.emit("")
    stamp.emit("Next (after reviewing the diff):")
    stamp.emit("  git add Codesys/ArfBot.project Codesys/ArfBot.xml")
    stamp.emit("  git commit -m \"Stamp PLC version {0}\"".format(tag))
    stamp.emit("  python Codesys\\Scripts\\TagPlcRelease.py")
    stamp.emit("  python Codesys\\Scripts\\TagPlcRelease.py --push")
    stamp.emit("Then open the PR into main and create the GitHub Release from that existing tag.")


def main():
    stamp.emit("CutPlcRelease starting")
    proj = stamp.ensure_project()
    stamp.emit("Project: {0}".format(proj.path))

    git_root = stamp.find_git_root(proj.path)
    stamp.emit("Git root: {0}".format(git_root))

    user_slug, dry_run, skip_confirm = parse_flags(extra_script_args())
    latest, ref = stamp.latest_tag_on_main(git_root)
    all_tags = list_tags(git_root)
    prefix = reltag.propose_prefix(all_tags, latest)
    stamp.emit("Proposed prefix: {0}".format(prefix))

    if user_slug:
        entered = user_slug
        stamp.emit("Slug/tag from argv: {0}".format(entered))
    elif ui_present():
        entered = ask_tag(prefix)
        stamp.emit("Slug/tag from dialog: {0}".format(entered))
    else:
        raise RuntimeError(
            "No slug given. Pass one after -- (headless) or run from "
            "Tools -> Scripting -> Execute Script File for the popup."
        )

    tag = reltag.resolve_user_tag(entered, prefix)
    reltag.validate_release_tag(tag, all_tags, max_len=stamp.MAX_VERSION_LEN)
    stamp.emit("sPlcVersion := '{0}'".format(tag))

    if dry_run:
        stamp.emit("Dry run: project not saved, XML not exported")
        follow_up_commands(tag)
        stamp.emit("script finished.")
        return

    if ui_present() and not skip_confirm:
        if not confirm_stamp(tag):
            raise RuntimeError("CutPlcRelease cancelled at confirm")

    gvl = stamp.find_textual(proj, "GVL_Version")
    if gvl is None or not gvl.has_textual_declaration:
        raise RuntimeError("GVL_Version not found or has no textual declaration")

    old = gvl.textual_declaration.text
    new = stamp.stamp_declaration(old, tag)
    if new == old:
        stamp.emit("GVL_Version.sPlcVersion already set")
    else:
        gvl.textual_declaration.replace(new_text=new)
        stamp.emit("Updated GVL_Version.sPlcVersion")

    proj.save()
    stamp.emit("Project saved")
    export_plcopen_xml(proj)
    follow_up_commands(tag)
    stamp.emit("script finished.")


try:
    main()
except Exception as exc:
    stamp.emit("ERROR: {0}".format(exc))
    stamp.emit(traceback.format_exc())
    try:
        if ui_present():
            system.ui.error("CutPlcRelease: {0}".format(exc))
    except Exception:
        pass
    stamp.write_log()
    stamp.maybe_exit(1)
else:
    stamp.write_log()
    stamp.maybe_exit(0)
