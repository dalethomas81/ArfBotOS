# Patch _00_Main.M_BuildTests with ST generated from BackupRetain.ret
from __future__ import print_function
import os
import sys
import traceback


def clean_arg(value):
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        return value[1:-1]
    return value


def log_path():
    if len(sys.argv) > 2 and sys.argv[2]:
        return clean_arg(sys.argv[2])
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "PatchBuildTests.out.txt")


LOG_LINES = []
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


def read_st(name):
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "st", name)
    handle = open(path, "r")
    text = handle.read()
    handle.close()
    if text.endswith("\n"):
        text = text[:-1]
    return text


def find_named(root, name):
    matches = root.find(name, recursive=True)
    if matches:
        return matches[0]
    return None


def replace_text(obj, kind, new_text):
    if kind == "decl":
        if not obj.has_textual_declaration:
            raise RuntimeError("{0} has no textual declaration".format(obj.get_name()))
        obj.textual_declaration.replace(new_text=new_text)
        return
    if not obj.has_textual_implementation:
        raise RuntimeError("{0} has no textual implementation".format(obj.get_name()))
    obj.textual_implementation.replace(new_text=new_text)


def ensure_project():
    project_path = clean_arg(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1] else None
    if projects.primary is not None:
        return projects.primary
    if not project_path:
        raise RuntimeError("No project is open and no project path was provided.")

    project_path = os.path.abspath(project_path)
    lock_path = os.path.splitext(project_path)[0] + ".~u"
    try:
        return projects.open(project_path, primary=True)
    except Exception as exc:
        emit("Primary open failed: {0}".format(exc))

    try:
        projects.open(project_path, primary=False, allow_readonly=True)
    except Exception as exc:
        emit("Readonly open failed: {0}".format(exc))

    if os.path.isfile(lock_path):
        try:
            os.remove(lock_path)
            emit("Removed stale lock: {0}".format(lock_path))
        except Exception as exc:
            emit("Could not remove lock: {0}".format(exc))

    proj = projects.open(project_path, primary=True)
    if proj is None:
        raise RuntimeError("Failed to open project: {0}".format(project_path))
    return proj


def main():
    emit("PatchBuildTests starting")
    proj = ensure_project()
    emit("Project: {0}".format(proj.path))

    main_pou = find_named(proj, "_00_Main")
    if main_pou is None:
        raise RuntimeError("_00_Main not found")
    build_tests = find_named(main_pou, "M_BuildTests")
    if build_tests is None:
        raise RuntimeError("M_BuildTests not found under _00_Main")

    new_impl = read_st("M_BuildTests_impl.st")
    replace_text(build_tests, "impl", new_impl)
    emit("Updated M_BuildTests implementation ({0} chars)".format(len(new_impl)))

    proj.save()
    emit("Project saved")
    emit("script finished.")


try:
    main()
except Exception as exc:
    emit("ERROR: {0}".format(exc))
    emit(traceback.format_exc())
    write_log()
    try:
        system.exit(1)
    except Exception:
        pass
else:
    write_log()
    try:
        system.exit(0)
    except Exception:
        pass
