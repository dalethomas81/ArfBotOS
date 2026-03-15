#!/usr/bin/env python3
"""
codesys_xml_normalize.py
========================
Git textconv driver for CodeSys PLCopen XML exports.

Normalizes a CodeSys XML file so that `git diff` produces clean,
human-readable output that focuses on real code/logic changes rather
than XML formatting noise.

What it does:
  - Consistent indentation (2 spaces)
  - Sorts XML attributes alphabetically (they have no defined order in XML)
  - Strips noise attributes that change every export (timestamps, GUIDs)
  - Normalizes whitespace in text content
  - Preserves all namespace declarations
  - Handles UTF-8 BOM gracefully

Usage (standalone):
  python codesys_xml_normalize.py <export.xml>

Git textconv setup (done automatically by setup_codesys_git_diff.bat):
  git config diff.codesys-xml.textconv "python path/to/codesys_xml_normalize.py"
  git config diff.codesys-xml.cachetextconv true
"""

import sys
import re
from xml.etree import ElementTree as ET
from xml.dom import minidom
from io import StringIO


# ---------------------------------------------------------------------------
# Attributes that change on every export and produce useless diff noise.
# Add more here if your project exports additional volatile metadata.
# ---------------------------------------------------------------------------
NOISE_ATTRIBUTES = {
    # PLCopen / CodeSys common volatiles
    "DateOfLastChange",
    "changeDate",
    "modificationDateTime",
    "creationDateTime",
    "lastChangeDate",
    # Some versions embed a build GUID or CRC per object
    "ObjectGuid",
    "checkSum",
    "buildNumber",
}

# Namespace URI → preferred prefix mapping (keeps output readable)
KNOWN_NAMESPACES = {
    "http://www.plcopen.org/xml/tc6_0200":  "",          # PLCopen default ns
    "http://www.plcopen.org/xml/tc6.xsd":   "",
    "http://www.3s-software.com/plcopenxml": "cs",        # CodeSys extension
    "http://www.w3.org/1999/xhtml":         "xhtml",
    "http://www.w3.org/2001/XMLSchema":     "xs",
    "http://www.w3.org/2001/XMLSchema-instance": "xsi",
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _register_namespaces(content: str) -> None:
    """Register all namespaces found in the document so ET preserves prefixes."""
    for match in re.finditer(r'xmlns(?::(\w+))?=["\']([^"\']+)["\']', content):
        prefix = match.group(1) or ""
        uri    = match.group(2)
        # Override with our known-pretty prefix if available
        pretty_prefix = KNOWN_NAMESPACES.get(uri, prefix)
        ET.register_namespace(pretty_prefix, uri)


def _local_name(tag: str) -> str:
    """Strip namespace URI from a tag, e.g. '{http://...}pou' → 'pou'."""
    return tag.split("}")[-1] if "}" in tag else tag


def _sort_and_clean_attribs(elem: ET.Element) -> None:
    """
    Sort attributes alphabetically and remove noise attributes.
    (ElementTree doesn't guarantee attribute order, so sorting gives
    a stable serialization.)
    """
    cleaned = {
        k: v
        for k, v in elem.attrib.items()
        if _local_name(k) not in NOISE_ATTRIBUTES
    }
    elem.attrib.clear()
    for k, v in sorted(cleaned.items()):
        elem.attrib[k] = v


def _normalize_text(text: str | None) -> str | None:
    """Strip leading/trailing whitespace; return None for empty strings."""
    if text is None:
        return None
    stripped = text.strip()
    return stripped if stripped else None


def _normalize_element(elem: ET.Element) -> None:
    """Recursively normalize an element tree."""
    _sort_and_clean_attribs(elem)

    elem.text = _normalize_text(elem.text)
    elem.tail = _normalize_text(elem.tail)

    for child in elem:
        _normalize_element(child)


def _pretty_print(raw_xml: str) -> str:
    """Convert a raw XML string to a nicely indented one-element-per-line form."""
    try:
        dom   = minidom.parseString(raw_xml.encode("utf-8"))
        ugly  = dom.toprettyxml(indent="  ", newl="\n", encoding=None)
    except Exception:
        # Fallback: return as-is
        return raw_xml

    lines = []
    for line in ugly.splitlines():
        # minidom adds its own <?xml?> declaration; skip it (we add our own)
        if line.startswith("<?xml"):
            continue
        # Drop blank lines minidom inserts between elements
        if not line.strip():
            continue
        lines.append(line)

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def normalize_file(filepath: str) -> None:
    """Read, normalize, and print a CodeSys XML export to stdout."""

    # --- Read with BOM / encoding tolerance ---
    content = None
    for enc in ("utf-8-sig", "utf-8", "latin-1"):
        try:
            with open(filepath, "r", encoding=enc) as fh:
                content = fh.read()
            break
        except (UnicodeDecodeError, OSError):
            continue

    if content is None:
        print(f"# ERROR: could not read file: {filepath}", file=sys.stderr)
        sys.exit(1)

    # --- Register namespaces BEFORE parsing so ET preserves prefixes ---
    _register_namespaces(content)

    # --- Parse ---
    try:
        root = ET.fromstring(content)
    except ET.ParseError as exc:
        # If we can't parse it, just dump the raw text so diff still works
        print(f"# WARNING: XML parse error ({exc}); showing raw content")
        print(content)
        return

    # --- Normalize ---
    _normalize_element(root)

    # --- Serialize ---
    raw = ET.tostring(root, encoding="unicode", xml_declaration=False)

    # --- Pretty-print ---
    pretty = _pretty_print(raw)

    # Emit with a stable XML declaration
    print('<?xml version="1.0" encoding="utf-8"?>')
    print(pretty)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <codesys_export.xml>", file=sys.stderr)
        sys.exit(1)
    normalize_file(sys.argv[1])
