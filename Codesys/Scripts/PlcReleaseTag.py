# CalVer helpers for ArfBotOS release tags.
#
# Tag shape: v{major}.{year}.{iso-week}.{n}-{Slug}
# Example:   v2.2026.38.0-RoiEditor
#
# Stdlib only so CPython can unit-test and preview a tag without CODESYS:
#   python Codesys\Scripts\PlcReleaseTag.py RoiEditor
from __future__ import print_function
import datetime
import re
import sys


TAG_RE = re.compile(
    r"^v(\d+)\.(\d{4})\.(\d{1,2})\.(\d+)(?:-([A-Za-z][A-Za-z0-9]*))?$"
)
SLUG_RE = re.compile(r"^[A-Za-z][A-Za-z0-9]*$")
MAX_VERSION_LEN = 80


def parse_tag(tag):
    if tag is None:
        return None
    text = tag.strip()
    match = TAG_RE.match(text)
    if not match:
        return None
    return {
        "major": int(match.group(1)),
        "year": int(match.group(2)),
        "week": int(match.group(3)),
        "n": int(match.group(4)),
        "slug": match.group(5) or "",
        "text": text,
    }


def iso_year_week(today=None):
    if today is None:
        today = datetime.date.today()
    iso = today.isocalendar()
    return int(iso[0]), int(iso[1])


def next_n(parsed_tags, major, year, week):
    numbers = [
        item["n"]
        for item in parsed_tags
        if item["major"] == major and item["year"] == year and item["week"] == week
    ]
    if not numbers:
        return 0
    return max(numbers) + 1


def format_tag(major, year, week, n, slug):
    return "v{0}.{1}.{2}.{3}-{4}".format(major, year, week, n, slug)


def propose_prefix(all_tags, latest_main_tag, today=None):
    year, week = iso_year_week(today)
    parsed_main = parse_tag(latest_main_tag) if latest_main_tag else None
    major = parsed_main["major"] if parsed_main else 2
    parsed_all = []
    for tag in all_tags or []:
        parsed = parse_tag(tag)
        if parsed is not None:
            parsed_all.append(parsed)
    n = next_n(parsed_all, major, year, week)
    return "v{0}.{1}.{2}.{3}-".format(major, year, week, n)


def resolve_user_tag(user_text, proposed_prefix):
    text = (user_text or "").strip()
    if not text:
        raise ValueError("A release slug or full tag is required")
    parsed = parse_tag(text)
    if parsed is not None:
        if not parsed["slug"]:
            raise ValueError("Tag '{0}' is missing a feature slug".format(text))
        return format_tag(
            parsed["major"],
            parsed["year"],
            parsed["week"],
            parsed["n"],
            parsed["slug"],
        )
    if SLUG_RE.match(text):
        return proposed_prefix + text
    raise ValueError(
        "Expected a feature slug or a full tag like {0}MyFeature, got '{1}'".format(
            proposed_prefix, text
        )
    )


def extract_plc_version(xml_text):
    if not xml_text:
        return ""
    match = re.search(
        r'<variable name="sPlcVersion">.*?<simpleValue value="([^"]*)"',
        xml_text,
        re.S,
    )
    if match:
        return match.group(1).strip().strip("'")
    match = re.search(r"sPlcVersion\s*:[^']*'([^']*)'", xml_text)
    return match.group(1).strip() if match else ""


def find_git_root(start):
    import os

    path = os.path.abspath(start)
    if os.path.isfile(path):
        path = os.path.dirname(path)
    while True:
        if os.path.isdir(os.path.join(path, ".git")) or os.path.isfile(
            os.path.join(path, ".git")
        ):
            return path
        parent = os.path.dirname(path)
        if parent == path:
            raise ValueError("No git repository found above {0}".format(start))
        path = parent


def validate_release_tag(tag, existing_tags, max_len=MAX_VERSION_LEN):
    parsed = parse_tag(tag)
    if parsed is None or not parsed["slug"]:
        raise ValueError("Not a release tag: '{0}'".format(tag))
    if len(tag) > max_len:
        raise ValueError(
            "Tag '{0}' is {1} chars; STRING({2}) max".format(tag, len(tag), max_len)
        )
    if tag in existing_tags:
        raise ValueError("Git tag '{0}' already exists".format(tag))
    return parsed


def _decode(output):
    if output is None:
        return ""
    if not isinstance(output, str):
        return output.decode("utf-8")
    return output


def _git(args, cwd):
    import subprocess

    output = subprocess.check_output(["git"] + args, cwd=cwd)
    return _decode(output).strip()


def preview_from_git(git_root, slug=None, today=None):
    latest = None
    for ref in ("main", "origin/main", "master", "origin/master"):
        try:
            latest = _git(["describe", "--tags", "--abbrev=0", ref], git_root)
            if latest:
                break
        except Exception:
            latest = None
    listing = _git(["tag", "--list"], git_root)
    all_tags = [line.strip() for line in listing.splitlines() if line.strip()]
    prefix = propose_prefix(all_tags, latest, today=today)
    if slug:
        tag = resolve_user_tag(slug, prefix)
        validate_release_tag(tag, all_tags)
        return latest, prefix, tag, all_tags
    return latest, prefix, None, all_tags


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    slug = argv[0] if argv else None
    import os

    git_root = find_git_root(sys.path[0] if sys.path else os.getcwd())
    latest, prefix, tag, _all_tags = preview_from_git(git_root, slug=slug)
    print("Latest tag on main: {0}".format(latest or "<none>"))
    print("Proposed prefix:    {0}".format(prefix))
    if tag:
        print("Release tag:        {0}".format(tag))
    else:
        print("Pass a slug to preview the full tag, e.g. RoiEditor")
    return 0


if __name__ == "__main__":
    sys.exit(main())
