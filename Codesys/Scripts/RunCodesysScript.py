from __future__ import print_function

import argparse
import os
import subprocess
import sys
import glob
import re


def default_project_path():
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    codesys_dir = os.path.dirname(scripts_dir)
    return os.path.join(codesys_dir, "ArfBot.project")


def default_output_path():
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(scripts_dir, "ListDeviceTree.out.txt")


def detect_codesys_exe():
    candidates = []

    env_path = os.environ.get("CODESYS_EXE")
    if env_path:
        candidates.append(env_path)

    candidates.extend(sorted(
        glob.glob(r"C:\Program Files\CODESYS *\CODESYS\Common\CODESYS.exe"),
        reverse=True,
    ))
    candidates.extend(sorted(
        glob.glob(r"C:\Program Files (x86)\CODESYS *\CODESYS\Common\CODESYS.exe"),
        reverse=True,
    ))

    candidates.append(r"C:\Program Files\CODESYS\CODESYS\Common\CODESYS.exe")
    candidates.append(r"C:\Program Files (x86)\CODESYS\CODESYS\Common\CODESYS.exe")

    seen = set()
    for candidate in candidates:
        if not candidate:
            continue
        normalized = os.path.normcase(os.path.abspath(candidate))
        if normalized in seen:
            continue
        seen.add(normalized)
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)

    return None


def codesys_install_root(codesys_exe):
    return os.path.abspath(os.path.join(os.path.dirname(codesys_exe), "..", ".."))


def version_to_profile_name(version_text):
    match = re.match(r"(?i)(\d+)\.(\d+)SP(\d+)(?:Patch(\d+))?", version_text.strip())
    if not match:
        return None

    major = match.group(1)
    minor = match.group(2)
    service_pack = match.group(3)
    patch = match.group(4)

    profile = "CODESYS V{0}.{1} SP{2}".format(major, minor, service_pack)
    if patch:
        profile += " Patch {0}".format(patch)
    return profile


def detect_codesys_profile(codesys_exe):
    install_root = codesys_install_root(codesys_exe)
    version_key_path = os.path.join(install_root, "VersionKey.ini")

    if os.path.isfile(version_key_path):
        with open(version_key_path, "r") as handle:
            for line in handle:
                if line.startswith("Version="):
                    profile = version_to_profile_name(line.split("=", 1)[1])
                    if profile:
                        return profile

    return None


def build_command(args):
    command = ['"{0}"'.format(args.codesys_exe)]
    scriptargs = list(args.scriptargs)

    if args.profile:
        command.append('--profile="{0}"'.format(args.profile))

    if args.project:
        command.append('--project="{0}"'.format(os.path.abspath(args.project)))

    command.append('--runscript="{0}"'.format(os.path.abspath(args.script)))

    if args.project:
        project_arg = os.path.abspath(args.project)
        if project_arg not in scriptargs:
            scriptargs.insert(0, project_arg)

    if args.output:
        output_arg = os.path.abspath(args.output)
        if output_arg not in scriptargs:
            insert_index = 1 if scriptargs else 0
            scriptargs.insert(insert_index, output_arg)

    if scriptargs:
        quoted_scriptargs = " ".join(['"{0}"'.format(arg) for arg in scriptargs])
        command.append("--scriptargs:{0}".format(quoted_scriptargs))

    if args.no_ui:
        command.append("--noUI")

    if args.text_prompts:
        command.append("--textPrompts")

    if args.enable_script_tracing:
        command.append("--enablescripttracing")

    if args.additional_folder:
        command.append('--additionalfolder="{0}"'.format(args.additional_folder))

    return command


def main():
    parser = argparse.ArgumentParser(
        description="Launch a CODESYS Python script through the CODESYS command line."
    )
    parser.add_argument(
        "--codesys-exe",
        default=None,
        help="Full path to CODESYS.exe; if omitted, the launcher tries to find it automatically",
    )
    parser.add_argument(
        "--script",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "ListDeviceTree.py"),
        help="Path to the CODESYS Python script to run",
    )
    parser.add_argument(
        "--project",
        default=default_project_path(),
        help="Path to the .project file to open first",
    )
    parser.add_argument(
        "--output",
        default=default_output_path(),
        help="Path to the text file written by the CODESYS script",
    )
    parser.add_argument(
        "--profile",
        default=None,
        help='Optional CODESYS profile, for example "CODESYS V3.5 SP19"',
    )
    parser.add_argument(
        "--additional-folder",
        default=None,
        help="Optional CODESYS additional folder argument",
    )
    parser.add_argument(
        "--no-ui",
        action="store_true",
        help="Run CODESYS without opening the main UI",
    )
    parser.add_argument(
        "--text-prompts",
        action="store_true",
        help="Show prompts in the console when available",
    )
    parser.add_argument(
        "--enable-script-tracing",
        action="store_true",
        help="Enable CODESYS script tracing output",
    )
    parser.add_argument(
        "scriptargs",
        nargs=argparse.REMAINDER,
        help="Arguments passed through to the CODESYS script",
    )

    args = parser.parse_args()

    if not args.codesys_exe:
        args.codesys_exe = detect_codesys_exe()

    if not args.profile and args.codesys_exe:
        args.profile = detect_codesys_profile(args.codesys_exe)

    if not os.path.isfile(args.codesys_exe):
        print("CODESYS executable not found.")
        print("Pass --codesys-exe explicitly or set CODESYS_EXE.")
        return 1

    if not os.path.isfile(args.script):
        print("Script not found: {0}".format(args.script))
        return 1

    if args.project and not os.path.isfile(args.project):
        print("Project not found: {0}".format(args.project))
        return 1

    command = build_command(args)

    print("Detected CODESYS executable: {0}".format(args.codesys_exe))
    if args.profile:
        print("Detected CODESYS profile: {0}".format(args.profile))
    command_line = " ".join(command)

    print("Running:")
    print(command_line)

    completed = subprocess.run(command_line, check=False)
    return completed.returncode


if __name__ == "__main__":
    sys.exit(main())
