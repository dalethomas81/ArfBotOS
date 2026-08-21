# Lists the CODESYS device tree for the primary project.
from __future__ import print_function
import os
import sys
import traceback


OUTPUT_LINES = []
ECHO_TO_CONSOLE = False


def emit(line):
    if ECHO_TO_CONSOLE:
        print(line)
    OUTPUT_LINES.append(line)


def safe_get_name(obj):
    try:
        return obj.get_name()
    except Exception:
        return "<unnamed>"


def safe_is_device(obj):
    try:
        return bool(obj.is_device)
    except Exception:
        return False


def safe_comm_details(device_obj):
    try:
        settings = device_obj.get_device_communication_settings()
        gateway = settings.gateway_guid
        address = settings.device_address
        if gateway or address:
            return " [gateway={0}, address={1}]".format(gateway, address)
    except Exception:
        pass
    return ""


def print_tree(node, depth):
    indent = "  " * depth
    name = safe_get_name(node)
    suffix = ""

    if safe_is_device(node):
        suffix = safe_comm_details(node)

    emit("{0}- {1}{2}".format(indent, name, suffix))

    for child in node.get_children():
        print_tree(child, depth + 1)


def default_output_path(project):
    if project is not None:
        try:
            project_dir = project.path.rsplit("\\", 1)[0]
            return project_dir + "\\Scripts\\ListDeviceTree.out.txt"
        except Exception:
            pass
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(scripts_dir, "ListDeviceTree.out.txt")


def default_error_path(project):
    if project is not None:
        try:
            project_dir = project.path.rsplit("\\", 1)[0]
            return project_dir + "\\Scripts\\ListDeviceTree.error.txt"
        except Exception:
            pass
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(scripts_dir, "ListDeviceTree.error.txt")


def resolve_output_path(project):
    if len(sys.argv) > 2 and sys.argv[2]:
        return sys.argv[2]
    return default_output_path(project)


def write_output_file(path):
    try:
        directory = os.path.dirname(path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        with open(path, "w") as handle:
            handle.write("\n".join(OUTPUT_LINES) + "\n")
        emit("Wrote device tree output to: {0}".format(path))
    except Exception as exc:
        emit("Failed to write output file: {0}".format(exc))


def write_error_file(path, exc):
    try:
        directory = os.path.dirname(path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        handle = open(path, "w")
        handle.write(str(exc) + "\n")
        handle.write(traceback.format_exc())
        handle.close()
    except Exception:
        pass


def get_project_candidates():
    try:
        return list(projects.all)
    except Exception:
        return []


def get_requested_project_path():
    if len(sys.argv) > 1 and sys.argv[1]:
        return sys.argv[1]
    return None


def should_echo_to_console():
    if len(sys.argv) > 3 and sys.argv[3]:
        value = str(sys.argv[3]).strip().lower()
        return value in ("1", "true", "yes", "on")
    return False


def ensure_project_open():
    requested_project = get_requested_project_path()
    if not requested_project:
        return None

    try:
        proj = projects.open(requested_project, primary=True)
        if proj is not None:
            try:
                system.delay(500)
            except Exception:
                pass
            return proj
    except Exception as exc:
        emit("Failed to open requested project: {0}".format(exc))

    return None


def wait_for_project():
    for _ in range(50):
        try:
            proj = projects.primary
            if proj is not None:
                return proj
        except Exception:
            pass

        candidates = get_project_candidates()
        if candidates:
            return candidates[0]

        try:
            system.delay(200)
        except Exception:
            pass

    return None


try:
    ECHO_TO_CONSOLE = should_echo_to_console()
    proj = wait_for_project()

    if proj is None:
        proj = ensure_project_open()
        if proj is None:
            proj = wait_for_project()

    if proj is None:
        emit("No primary project is open.")
        candidates = get_project_candidates()
        emit("Open project count: {0}".format(len(candidates)))
    else:
        top_level_devices = [child for child in proj.get_children() if safe_is_device(child)]

        if not top_level_devices:
            emit("No top-level device nodes were found in the primary project.")
        else:
            emit("Device tree for project: {0}".format(proj.path))
            for device in top_level_devices:
                print_tree(device, 0)

    write_output_file(resolve_output_path(proj))
    emit("script finished.")
except Exception as exc:
    try:
        write_error_file(default_error_path(projects.primary), exc)
    except Exception:
        pass

try:
    system.exit(0)
except Exception:
    pass
