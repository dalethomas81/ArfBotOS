# Probe Visualization Manager / visu tree and create an Animator visualization.
# HTML5 control install is still Tools -> Visualization Element Repository
# (or HTML5 Control Editor -> Save and Install). This script prepares the project.
from __future__ import print_function
import os
import sys
import traceback

OUTPUT_LINES = []


def emit(line):
    OUTPUT_LINES.append(line)
    try:
        print(line)
    except Exception:
        pass


def write_output(path):
    folder = os.path.dirname(path)
    if folder and not os.path.isdir(folder):
        os.makedirs(folder)
    with open(path, "w") as handle:
        handle.write("\n".join(OUTPUT_LINES))
        handle.write("\n")


def find_visual(root, name):
    matches = root.find(name, recursive=True)
    if not matches:
        return None
    for obj in matches:
        if getattr(obj, "is_visualobject", False):
            return obj
    return None


def find_named(root, name):
    matches = root.find(name, recursive=True)
    if not matches:
        return None
    return matches[0]


def dump_enum(enum_type, title):
    emit("=== {0} ===".format(title))
    try:
        names = dir(enum_type)
    except Exception, exc:
        emit("dir failed: {0}".format(exc))
        return
    for name in names:
        if name.startswith("_"):
            continue
        try:
            value = getattr(enum_type, name)
            emit("  {0} = {1}".format(name, value))
        except Exception:
            emit("  {0}".format(name))


def dump_object(obj, title, max_attrs=80):
    emit("=== {0} ({1}) ===".format(title, type(obj).__name__))
    try:
        names = dir(obj)
    except Exception, exc:
        emit("dir failed: {0}".format(exc))
        return
    count = 0
    for name in names:
        if name.startswith("_"):
            continue
        count += 1
        if count > max_attrs:
            emit("  ... truncated")
            break
        try:
            value = getattr(obj, name)
            text = str(value)
            if len(text) > 180:
                text = text[:180] + "..."
            emit("  {0} = {1}".format(name, text))
        except Exception, exc:
            emit("  {0} = <err {1}>".format(name, exc))


def default_output_path():
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(scripts_dir, "temp", "InstallRobotAnimator.out.txt")


def main():
    out_path = default_output_path()
    if len(sys.argv) >= 3 and sys.argv[2] and not sys.argv[2].startswith("--"):
        out_path = sys.argv[2]

    proj = projects.primary
    if proj is None and len(sys.argv) >= 2 and sys.argv[1].lower().endswith(".project"):
        emit("primary missing; opening {0}".format(sys.argv[1]))
        proj = projects.open(sys.argv[1])
    if proj is None:
        emit("no primary project")
        write_output(out_path)
        system.exit(1)
        return

    emit("project: {0}".format(proj.path))

    try:
        dump_enum(VisualElementType, "VisualElementType")
    except Exception, exc:
        emit("VisualElementType missing: {0}".format(exc))

    app = None
    found = proj.find("Application", recursive=True)
    if found:
        app = found[0]
        emit("application: {0}".format(app.get_name()))

    visus = []
    if app is not None:
        def walk(node, depth):
            name = "<unnamed>"
            try:
                name = node.get_name()
            except Exception:
                pass
            flag = ""
            try:
                if getattr(node, "is_visualobject", False):
                    flag = " [VISU]"
                    visus.append(node)
            except Exception:
                pass
            emit("{0}- {1}{2}".format("  " * depth, name, flag))
            try:
                children = node.get_children()
            except Exception:
                children = []
            for child in children:
                walk(child, depth + 1)
        emit("=== Application tree ===")
        walk(app, 0)

    vm = find_named(proj, "VisualizationManager")
    if vm is None:
        vm = find_named(proj, "Visualization Manager")
    if vm is None:
        emit("Visualization Manager not found by name")
    else:
        dump_object(vm, "Visualization Manager")
        for attr in (
            "support_client_animations",
            "SupportClientAnimations",
            "client_animations",
            "overlay_native_elements",
            "SupportOverlayOfNativeElements",
        ):
            try:
                emit("try setattr {0}".format(attr))
                setattr(vm, attr, True)
                emit("  set {0} ok, now {1}".format(attr, getattr(vm, attr)))
            except Exception, exc:
                emit("  {0}: {1}".format(attr, exc))

    animator = find_visual(proj, "Animator")
    created = False
    if animator is None and app is not None:
        try:
            animator = app.create_visualobject("Animator")
            created = True
            emit("created visualization Animator")
        except Exception, exc:
            emit("create_visualobject Animator failed: {0}".format(exc))
            emit(traceback.format_exc())

    if animator is not None:
        try:
            animator.begin_modify()
            lst = animator.visual_element_list
            existing = None
            for i in range(len(lst)):
                el = lst[i]
                try:
                    if el.ReadOnlyElement.InstanceName == "AnimatorHint":
                        existing = el
                        break
                except Exception:
                    pass
            if existing is None:
                existing = lst.add_element(VisualElementType.Rectangle)
                existing.ReadOnlyElement.InstanceName = "AnimatorHint"
            existing.set_property("Position.X", 16)
            existing.set_property("Position.Y", 16)
            existing.set_property("Position.Width", 640)
            existing.set_property("Position.Height", 48)
            existing.set_property("Texts.Text", "Robot Animator: install HTML5 control, then drop RobotAnimator here. Bind J1..J6 to IoConfig_Globals.Jx.fActPosition")
            existing.set_property("Text properties.Horizontal alignment", "LEFT")
            existing.set_property("Color variables.Normal state.Fill color", "16#FF171F2A")
            existing.set_property("Color variables.Normal state.Frame color", "16#FF2EE6D6")
            animator.end_modify()
            emit("patched Animator visu (created={0})".format(created))
        except Exception, exc:
            emit("Animator visu patch failed: {0}".format(exc))
            emit(traceback.format_exc())
            try:
                animator.end_modify()
            except Exception:
                pass

    try:
        proj.save()
        emit("saved project")
    except Exception, exc:
        emit("save failed: {0}".format(exc))

    write_output(out_path)
    system.exit(0)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        out_path = default_output_path()
        if len(sys.argv) >= 3 and sys.argv[2] and not sys.argv[2].startswith("--"):
            out_path = sys.argv[2]
        emit("UNHANDLED")
        emit(traceback.format_exc())
        write_output(out_path)
        try:
            system.exit(1)
        except Exception:
            sys.exit(1)
