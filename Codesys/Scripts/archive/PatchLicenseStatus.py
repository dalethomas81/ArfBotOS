# Add FB_LicenseStatus, wire it in GVL/_00_Main, and add Component Manager + CmpEventMgr.
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
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "PatchLicenseStatus.out.txt")


LOG_LINES = []
LOG_FILE = log_path()
ST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "st")


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
    path = os.path.join(ST_DIR, name)
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


def replace_text(obj, kind, new_text):
    if kind == "decl":
        if not obj.has_textual_declaration:
            raise RuntimeError("{0} has no textual declaration".format(obj.get_name()))
        obj.textual_declaration.replace(new_text=new_text)
        return
    if not obj.has_textual_implementation:
        raise RuntimeError("{0} has no textual implementation".format(obj.get_name()))
    obj.textual_implementation.replace(new_text=new_text)


def get_text(obj, kind):
    if kind == "decl":
        return obj.textual_declaration.text
    return obj.textual_implementation.text


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


def ensure_library(libman, display_name, namespace, placeholder=False, default_resolution=None):
    existing = libman.get_libraries(recursive=False)
    emit("Libraries: {0}".format(existing))
    already = any(display_name.split(",")[0].strip().lower() in str(e).lower() for e in existing)
    if already:
        emit("Library already present: {0}".format(display_name))
    else:
        emit("Adding library: {0}".format(display_name))
        if placeholder:
            libman.add_placeholder(display_name, default_resolution or display_name)
        else:
            libman.add_library(display_name)

    for ref in libman.references:
        name = str(ref.name)
        if display_name.split(",")[0].strip().lower() not in name.lower() and display_name.lower() not in name.lower():
            if placeholder and ("#" + display_name).lower() not in name.lower():
                continue
            if not placeholder:
                continue
        try:
            emit("Ref {0} namespace={1} qualified_only={2}".format(name, ref.namespace, ref.qualified_only))
        except Exception as exc:
            emit("Ref {0} inspect failed: {1}".format(name, exc))
            continue
        if namespace and ref.namespace != namespace:
            ref.namespace = namespace
            emit("Set namespace {0} on {1}".format(namespace, name))
        if not ref.qualified_only:
            ref.qualified_only = True
            emit("Set qualified_only=True on {0}".format(name))


def ensure_method(fb, name, return_type, decl_file, impl_file):
    method = find_named(fb, name)
    if method is None:
        emit("Creating method {0}".format(name))
        if return_type:
            method = fb.create_method(name=name, return_type=return_type)
        else:
            method = fb.create_method(name=name)
    else:
        emit("Method exists: {0}".format(name))
    replace_text(method, "decl", read_st(decl_file))
    replace_text(method, "impl", read_st(impl_file))


def insert_before_last_end_var(decl, snippet):
    if snippet.strip() in decl:
        return decl, False
    idx = decl.rfind("END_VAR")
    if idx < 0:
        raise RuntimeError("No END_VAR in declaration")
    return decl[:idx] + snippet + decl[idx:], True


def collect_build_messages():
    try:
        msgs = system.get_message_objects()
    except Exception:
        try:
            msgs = system.get_messages()
        except Exception as exc:
            emit("Could not read messages: {0}".format(exc))
            return
    count = 0
    errors = 0
    for msg in msgs:
        try:
            sev = str(msg.severity)
            text = msg.text
        except Exception:
            sev = "?"
            text = "<unprintable>"
        sev_l = sev.lower()
        if "error" not in sev_l and "warning" not in sev_l and "fatal" not in sev_l:
            continue
        count += 1
        if "error" in sev_l or "fatal" in sev_l:
            errors += 1
        if count > 80:
            emit("... truncated compile message list")
            break
        emit("COMPILE {0}: {1}".format(sev, text))
    emit("Compile errors: {0}".format(errors))


def main():
    emit("PatchLicenseStatus starting")
    proj = ensure_project()
    emit("Project: {0}".format(proj.path))

    app = find_named(proj, "Application")
    if app is None:
        raise RuntimeError("Application not found")

    libman = None
    try:
        libman = app.get_library_manager()
        emit("Using application library manager")
    except Exception as exc:
        emit("app.get_library_manager failed: {0}".format(exc))
        libman = proj.get_library_manager()
        emit("Using project library manager")

    ensure_library(libman, "Component Manager, * (System)", "CmpMgr")
    ensure_library(libman, "CmpEventMgr, * (System)", "CmpEventMgr")

    pou_folder = find_named(proj, "POU")
    if pou_folder is None:
        pou_folder = app
        emit("POU folder missing, creating FB under Application")

    fb = find_named(proj, "FB_LicenseStatus")
    if fb is None:
        emit("Creating FB_LicenseStatus")
        fb = pou_folder.create_pou(
            name="FB_LicenseStatus",
            type=PouType.FunctionBlock,
            interfaces="CmpEventMgr.ICmpEventCallback",
        )
    else:
        emit("FB_LicenseStatus already exists")

    replace_text(fb, "decl", read_st("FB_LicenseStatus_decl.st"))
    replace_text(fb, "impl", read_st("FB_LicenseStatus_impl.st"))

    ensure_method(fb, "EventCallback", "SysTypes.RTS_IEC_RESULT",
                  "FB_LicenseStatus_EventCallback_decl.st",
                  "FB_LicenseStatus_EventCallback_impl.st")
    ensure_method(fb, "FB_Init", "BOOL",
                  "FB_LicenseStatus_FB_Init_decl.st",
                  "FB_LicenseStatus_FB_Init_impl.st")
    ensure_method(fb, "FB_Exit", "BOOL",
                  "FB_LicenseStatus_FB_Exit_decl.st",
                  "FB_LicenseStatus_FB_Exit_impl.st")
    ensure_method(fb, "M_EnsureRegistered", None,
                  "FB_LicenseStatus_M_EnsureRegistered_decl.st",
                  "FB_LicenseStatus_M_EnsureRegistered_impl.st")
    ensure_method(fb, "M_Unregister", None,
                  "FB_LicenseStatus_M_Unregister_decl.st",
                  "FB_LicenseStatus_M_Unregister_impl.st")
    ensure_method(fb, "M_UpdateRemaining", None,
                  "FB_LicenseStatus_M_UpdateRemaining_decl.st",
                  "FB_LicenseStatus_M_UpdateRemaining_impl.st")
    ensure_method(fb, "M_BuildBanner", None,
                  "FB_LicenseStatus_M_BuildBanner_decl.st",
                  "FB_LicenseStatus_M_BuildBanner_impl.st")

    gvl = find_textual(proj, "GVL")
    if gvl is None:
        raise RuntimeError("GVL not found")
    emit("Using GVL object type={0}".format(gvl.type))
    gvl_decl = get_text(gvl, "decl")
    new_gvl, changed = insert_before_last_end_var(
        gvl_decl,
        "\tLicenseStatus: FB_LicenseStatus;\n",
    )
    if changed:
        replace_text(gvl, "decl", new_gvl)
        emit("Added LicenseStatus instance to GVL")
    else:
        emit("GVL already has LicenseStatus")

    main_pou = find_textual(proj, "_00_Main")
    if main_pou is None:
        raise RuntimeError("_00_Main not found")
    main_impl = get_text(main_pou, "impl")
    if "GVL.LicenseStatus" not in main_impl:
        needle = "GVL.LicenseStatus();\n"
        marker = "\tIF FirstScan THEN"
        if marker in main_impl:
            main_impl = main_impl.replace(
                marker,
                "\tGVL.LicenseStatus();\n\n" + marker,
                1,
            )
        else:
            main_impl = "\tGVL.LicenseStatus();\n" + main_impl
        replace_text(main_pou, "impl", main_impl)
        emit("Added GVL.LicenseStatus() call to _00_Main")
    else:
        emit("_00_Main already calls LicenseStatus")

    proj.save()
    emit("Project saved")

    try:
        emit("Building application...")
        app.build()
        collect_build_messages()
        emit("Build finished")
    except Exception as exc:
        emit("Build failed: {0}".format(exc))
        emit(traceback.format_exc())
        collect_build_messages()

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
