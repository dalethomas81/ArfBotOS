# Stamp GVL_Version.sPlcVersion from git.
#
# Tag comes from main (where releases are tagged). The commit and dirty flag
# come from the current HEAD (dev, a feature branch, etc.).
#
# Format matches git describe: <tag>-<n>-g<sha>[-dirty]
#
# Run from CODESYS: Tools -> Scripting -> Execute Script File
# Or: python Codesys\Scripts\RunCodesysScript.py --script Codesys\Scripts\StampPlcVersion.py --project Codesys\ArfBot.project --no-ui --text-prompts
from __future__ import print_function
import os
import re
import sys
import traceback

from System.Diagnostics import Process, ProcessStartInfo


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


def quote_arg(arg):
    arg = str(arg)
    if not arg or any(ch in arg for ch in ' \t"'):
        return '"' + arg.replace('"', '\\"') + '"'
    return arg


def run_git(args, cwd, allowed_codes=(0,)):
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
    if code not in allowed_codes:
        raise RuntimeError("git {0} failed in {1} (exit {2}): {3}".format(
            " ".join(args), cwd, code, err or out or "no output"))
    return out, code


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


def bind_host(host=None):
    # CODESYS injects projects/system only into the executed script, not imports.
    if host is None:
        host = sys.modules.get("__main__")
    if host is None:
        return
    g = globals()
    for name in ("projects", "system"):
        if hasattr(host, name):
            g[name] = getattr(host, name)


def launched_headless():
    return len(sys.argv) > 1 and str(sys.argv[1]).lower().endswith(".project")


def maybe_exit(code):
    if launched_headless() and not ALREADY_OPEN:
        try:
            system.exit(code)
        except Exception:
            pass


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
    bind_host()
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
            tag, _code = run_git(["describe", "--tags", "--abbrev=0", ref], git_root)
            if tag:
                emit("Tag from {0}: {1}".format(ref, tag))
                return tag, ref
        except Exception as exc:
            last_err = exc
            emit("No tag on {0}: {1}".format(ref, exc))
    raise RuntimeError("Could not read a tag from main ({0})".format(last_err))


def is_dirty(git_root):
    porcelain, _code = run_git(["status", "--porcelain", "-uno"], git_root)
    return porcelain != ""


def describe_version(git_root):
    tag, ref = latest_tag_on_main(git_root)
    sha, _code = run_git(["rev-parse", "--short", "HEAD"], git_root)
    branch, _code = run_git(["rev-parse", "--abbrev-ref", "HEAD"], git_root)
    count, _code = run_git(["rev-list", "--count", "{0}..HEAD".format(tag)], git_root)
    dirty = is_dirty(git_root)

    _out, ancestor_code = run_git(
        ["merge-base", "--is-ancestor", tag, "HEAD"],
        git_root,
        allowed_codes=(0, 1),
    )
    if ancestor_code != 0:
        emit("Note: tag {0} is not in this branch history. Count {1} is commits on HEAD not reachable from that tag.".format(tag, count))

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


def run():
    try:
        main()
    except Exception as exc:
        emit("ERROR: {0}".format(exc))
        emit(traceback.format_exc())
        write_log()
        maybe_exit(1)
        return
    write_log()
    maybe_exit(0)


if __name__ == "__main__":
    run()
