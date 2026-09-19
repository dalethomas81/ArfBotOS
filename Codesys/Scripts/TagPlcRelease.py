# Create an annotated git tag from GVL_Version.sPlcVersion in Codesys/ArfBot.xml.
#
# Run AFTER committing the CutPlcRelease stamp (the XML in HEAD is the source of truth):
#   Tools -> Scripting -> Execute Script File -> TagPlcRelease.py
#   python Codesys\Scripts\TagPlcRelease.py
#   python Codesys\Scripts\TagPlcRelease.py --dry-run
#   python Codesys\Scripts\TagPlcRelease.py --no-push
#
# IronPython (CODESYS) must not use subprocess — that raises
# DeprecationWarning: sys.exc_clear() not supported in 3.x as Script Messages errors.
from __future__ import print_function
import argparse
import os
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)
import PlcReleaseTag as reltag


XML_REL = "Codesys/ArfBot.xml"
PROJECT_REL = "Codesys/ArfBot.project"
STAGED_PATHS = (XML_REL, PROJECT_REL)


def running_in_codesys():
    host = sys.modules.get("__main__")
    return host is not None and hasattr(host, "projects")


def quote_arg(arg):
    arg = str(arg)
    if not arg or any(ch in arg for ch in ' \t"'):
        return '"' + arg.replace('"', '\\"') + '"'
    return arg


def git_via_process(args, cwd, check=True):
    from System.Diagnostics import Process, ProcessStartInfo

    psi = ProcessStartInfo()
    psi.FileName = "git.exe"
    psi.Arguments = " ".join([quote_arg(a) for a in args])
    psi.WorkingDirectory = cwd
    psi.RedirectStandardOutput = True
    psi.RedirectStandardError = True
    psi.UseShellExecute = False
    psi.CreateNoWindow = True

    proc = Process()
    proc.StartInfo = psi
    if not proc.Start():
        raise RuntimeError("Could not start git.exe")
    out = proc.StandardOutput.ReadToEnd().strip()
    err = proc.StandardError.ReadToEnd().strip()
    proc.WaitForExit()
    code = proc.ExitCode
    proc.Close()
    if check and code != 0:
        raise RuntimeError(
            "git {0} failed (exit {1}): {2}".format(
                " ".join(args), code, err or out or "no output"
            )
        )
    return out, code


def git_via_subprocess(args, cwd, check=True):
    import subprocess

    process = subprocess.Popen(
        ["git"] + list(args),
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    out, err = process.communicate()
    out = reltag._decode(out).strip()
    err = reltag._decode(err).strip()
    if check and process.returncode != 0:
        raise RuntimeError(
            "git {0} failed (exit {1}): {2}".format(
                " ".join(args), process.returncode, err or out or "no output"
            )
        )
    return out, process.returncode


def git(args, cwd, check=True):
    try:
        return git_via_process(args, cwd, check)
    except ImportError:
        return git_via_subprocess(args, cwd, check)


def porcelain_for(git_root, paths):
    out, _code = git(["status", "--porcelain", "--"] + list(paths), git_root)
    return out


def list_tags(git_root):
    out, _code = git(["tag", "--list"], git_root)
    return [line.strip() for line in out.splitlines() if line.strip()]


def head_sha(git_root):
    out, _code = git(["rev-parse", "HEAD"], git_root)
    return out


def short_sha(git_root):
    out, _code = git(["rev-parse", "--short", "HEAD"], git_root)
    return out


def xml_at_head(git_root):
    out, code = git(["show", "HEAD:{0}".format(XML_REL)], git_root, check=False)
    if code != 0:
        raise RuntimeError(
            "HEAD has no {0}. Commit the CutPlcRelease stamp first.".format(XML_REL)
        )
    return out


def tag_commit(git_root, tag):
    out, code = git(
        ["rev-parse", "--verify", "--quiet", "{0}^{{commit}}".format(tag)],
        git_root,
        check=False,
    )
    if code != 0 or not out:
        return None
    return out


def require_committed_stamp(git_root):
    dirty = porcelain_for(git_root, STAGED_PATHS)
    if dirty:
        raise RuntimeError(
            "Commit the stamped PLC files first. Uncommitted:\n{0}".format(dirty)
        )


def tag_from_head_xml(git_root):
    text = xml_at_head(git_root)
    version = reltag.extract_plc_version(text)
    if not version:
        raise RuntimeError("GVL_Version.sPlcVersion not found in HEAD {0}".format(XML_REL))
    reltag.validate_release_tag(version, [], max_len=reltag.MAX_VERSION_LEN)
    existing = list_tags(git_root)
    pointed = tag_commit(git_root, version)
    if pointed is None:
        reltag.validate_release_tag(version, existing, max_len=reltag.MAX_VERSION_LEN)
    return version, pointed


def create_tag(git_root, tag):
    git(["tag", "-a", tag, "-m", tag], git_root)


def push_tag(git_root, tag):
    git(["push", "origin", "refs/tags/{0}".format(tag)], git_root)


def cli_argv(argv):
    if argv is None:
        argv = sys.argv[1:]
    filtered = []
    for raw in argv:
        value = raw.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
            value = value[1:-1]
        lower = value.lower()
        if not value or value == "--":
            continue
        if lower.endswith(".project") or lower.endswith(".txt"):
            continue
        filtered.append(value)
    return filtered


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Tag HEAD with sPlcVersion from the committed Codesys/ArfBot.xml."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the tag and checks; do not create or push it",
    )
    parser.add_argument(
        "--no-push",
        action="store_true",
        help="Create the local tag only; do not push to origin",
    )
    args = parser.parse_args(cli_argv(argv))
    do_push = not args.dry_run and not args.no_push

    git_root = reltag.find_git_root(_SCRIPT_DIR)
    require_committed_stamp(git_root)
    tag, pointed = tag_from_head_xml(git_root)
    head = head_sha(git_root)
    print("HEAD:  {0} ({1})".format(short_sha(git_root), head[:12]))
    print("Stamp: {0}".format(tag))

    if pointed is not None and pointed != head:
        raise RuntimeError(
            "Tag {0} already exists on {1}, not HEAD ({2})".format(
                tag, pointed[:12], head[:12]
            )
        )

    if args.dry_run:
        if pointed == head:
            print("Tag {0} already points at HEAD.".format(tag))
        else:
            print("Dry run: would create annotated tag {0} on HEAD".format(tag))
        print("Dry run: would push origin {0}".format(tag))
        return 0

    if pointed == head:
        print("Tag {0} already points at HEAD.".format(tag))
    else:
        create_tag(git_root, tag)
        print("Created annotated tag {0}".format(tag))

    if do_push:
        print("Pushing {0} to origin...".format(tag))
        push_tag(git_root, tag)
        print("Pushed origin {0}".format(tag))
    else:
        print("Local tag only. Push later with: git push origin {0}".format(tag))
    return 0


if __name__ == "__main__":
    code = 0
    try:
        code = main()
    except Exception as exc:
        print("ERROR: {0}".format(exc))
        code = 1
    if not running_in_codesys():
        sys.exit(code)
