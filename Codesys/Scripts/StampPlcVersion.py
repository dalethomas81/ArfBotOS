# Stamp GVL_Version.sPlcVersion from git.
#
# Tag comes from main (where releases are tagged). The commit and dirty flag
# come from the current HEAD (dev, a feature branch, etc.).
#
# Format matches git describe: <tag>-<n>-g<sha>[-dirty]
#
# Run from CODESYS: Tools → Scripting → Execute Script File
# Or: python Codesys\Scripts\RunCodesysScript.py --script Codesys\Scripts\StampPlcVersion.py --project Codesys\ArfBot.project --no-ui --text-prompts
from __future__ import print_function
import os
import re
import subprocess
import sys
import traceback


MAX_VERSION_LEN = 80
TAG_REFS = ("main", "origin/main", "master", "origin/master")
ALREADY_OPEN = False
LOG_LINES = []


def clean_arg(value):
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        return value[1:-1]
    return value


def log_path():
    if len(sys.argv) > 2 and sys.argv[2]:
        return clean_arg(sys.argv[2])
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "StampPlcVersion.out.txt")


LOG_FILE = log_path()


def emit(line):
    print(line)
    LOG_LINES.append(line)


def write_log():
    directory = os.path.dirname(LOG_FILE)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    handle = open(LOG_FILE, "w")
    handle.write("\n".join(LOG_LINES) + "\n")
    handle.close()


def decode_output(raw):
    if raw is None:
        return ""
    if not isinstance(raw, str):
        try:
            raw = raw.decode("utf-8")
        except Exception:
            raw = str(raw)
    return raw.strip()


def run_git(args, cwd):
    last_err = ""
    for exe in ("git", "git.exe"):
        try:
            proc = subprocess.Popen(
                [exe] + list(args),
                cwd=cwd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            out, err = proc.communicate()
            out = decode_output(out)
            err = decode_output(err)
            if proc.returncode == 0:
                return out
            last_err = err or out or "exit {0}".format(proc.returncode)
        except Exception as exc:
            last_err = str(exc)
    raise RuntimeError("git {0} failed in {1}: {2}".format(" ".join(args), cwd, last_err))


def find_git_root(start):
    path = os.path.abspath(start)
    if os.path.isfile(path):
        path = os.path.dirname(path)
    while True:
        if os.path.isdir(os.path.join(path, ".git")) or os.path.isfile(os.path.join(path, ".git")):
            return path
        parent = os.path.dirname(path)
        if parent == path:
            raise RuntimeError("No git repository found above {0}".format(start))
        path = parent


def find_textual(root, name):
    matches = root.find(name, recursive=True)
    if not matches:
        return None
    for obj in matches:
        try:
            if obj.has_textual_declaration:
                return obj
        except Exception:
            pass
    return matches[0]


def ensure_project():
    global ALREADY_OPEN
    if projects.primary is not None:
        ALREADY_OPEN = True
        return projects.primary
    if len(sys.argv) < 2 or not sys.argv[1]:
        raise RuntimeError("No project is open and no project path was provided.")
    project_path = os.path.abspath(clean_arg(sys.argv[1]))
    proj = projects.open(project_path, primary=True)
    if proj is None:
        raise RuntimeError("Failed to open project: {0}".format(project_path))
    return proj


def latest_tag_on_main(git_root):
    last_err = None
    for ref in TAG_REFS:
        try:
            tag = run_git(["describe", "--tags", "--abbrev=0", ref], git_root)
            if tag:
                emit("Tag from {0}: {1}".format(ref, tag))
                return tag, ref
        except Exception as exc:
            last_err = exc
            emit("No tag on {0}: {1}".format(ref, exc))
    raise RuntimeError("Could not read a tag from main ({0})".format(last_err))


def is_dirty(git_root):
    porcelain = run_git(["status", "--porcelain", "-uno"], git_root)
    return porcelain != ""


def describe_version(git_root):
    tag, ref = latest_tag_on_main(git_root)
    sha = run_git(["rev-parse", "--short", "HEAD"], git_root)
    branch = run_git(["rev-parse", "--abbrev-ref", "HEAD"], git_root)
    count = run_git(["rev-list", "--count", "{0}..HEAD".format(tag)], git_root)
    dirty = is_dirty(git_root)

    try:
        run_git(["merge-base", "--is-ancestor", tag, "HEAD"], git_root)
    except Exception:
        emit("WARNING: tag {0} is not an ancestor of HEAD. Count {1} may be misleading.".format(tag, count))

    if count == "0":
        version = tag
    else:
        version = "{0}-{1}-g{2}".format(tag, count, sha)
    if dirty:
        version = version + "-dirty"

    emit("Branch: {0}".format(branch))
    emit("HEAD: {0}".format(sha))
    emit("Commits since {0} ({1}): {2}".format(tag, ref, count))
    emit("Dirty: {0}".format(dirty))
    return version


def stamp_declaration(text, version):
    quoted = version.replace("'", "")
    pattern = r"(sPlcVersion\s*:\s*STRING(?:\s*\(\s*\d+\s*\))?\s*:=\s*)'[^']*'"
    new_text, n = re.subn(pattern, r"\g<1>'{0}'".format(quoted), text, count=1)
    if n == 1:
        return new_text
    pattern_bare = r"(sPlcVersion\s*:\s*STRING(?:\s*\(\s*\d+\s*\))?\s*;)"
    new_text, n = re.subn(
        pattern_bare,
        "sPlcVersion : STRING({0}) := '{1}';".format(MAX_VERSION_LEN, quoted),
        text,
        count=1,
    )
    if n == 1:
        return new_text
    raise RuntimeError("Could not find sPlcVersion in GVL_Version declaration")


def main():
    emit("StampPlcVersion starting")
    proj = ensure_project()
    emit("Project: {0}".format(proj.path))

    git_root = find_git_root(proj.path)
    emit("Git root: {0}".format(git_root))

    version = describe_version(git_root)
    if len(version) > MAX_VERSION_LEN:
        raise RuntimeError("Version '{0}' is {1} chars; STRING({2}) max".format(version, len(version), MAX_VERSION_LEN))
    emit("sPlcVersion := '{0}'".format(version))

    gvl = find_textual(proj, "GVL_Version")
    if gvl is None or not gvl.has_textual_declaration:
        raise RuntimeError("GVL_Version not found or has no textual declaration")

    old = gvl.textual_declaration.text
    new = stamp_declaration(old, version)
    if new == old:
        emit("GVL_Version.sPlcVersion already set")
    else:
        gvl.textual_declaration.replace(new_text=new)
        emit("Updated GVL_Version.sPlcVersion")

    proj.save()
    emit("Project saved")
    emit("script finished.")


try:
    main()
except Exception as exc:
    emit("ERROR: {0}".format(exc))
    emit(traceback.format_exc())
    write_log()
    if not ALREADY_OPEN:
        try:
            system.exit(1)
        except Exception:
            pass
else:
    write_log()
    if not ALREADY_OPEN:
        try:
            system.exit(0)
        except Exception:
            pass
