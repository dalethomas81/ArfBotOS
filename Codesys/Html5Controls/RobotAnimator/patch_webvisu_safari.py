#!/usr/bin/env python3
"""Patch CODESYS webvisu.js so HTML5 overlay iframes run in Safari.

Visualization 4.10 creates overlay iframes with src=about:blank, then srcdoc,
sandbox=allow-scripts (unique origin), and a csp= attribute. Safari often
leaves that iframe blank; Chromium does not.

This file is overwritten on every application download. Re-run after download.
"""
from pathlib import Path
import shutil

P = Path("/var/opt/codesys/PlcLogic/visu/webvisu.js")
BACKUP = Path("/var/opt/codesys/PlcLogic/visu/webvisu.js.pre-safari-patch")

OLD_SRC = 'f.setAttribute("src","about:blank");'
OLD_SANDBOX = 'f.setAttribute("sandbox","allow-scripts");'
NEW_SANDBOX = 'f.setAttribute("sandbox","allow-scripts allow-same-origin");'
OLD_CSP = 'f.setAttribute("csp",b);'


def main():
    t = P.read_text(encoding="utf-8", errors="replace")
    if "allow-scripts allow-same-origin" in t and OLD_SRC not in t:
        print("already patched:", P)
        return
    if not BACKUP.exists():
        shutil.copy2(P, BACKUP)
        print("backup", BACKUP)
    n = t
    if OLD_SRC in n:
        n = n.replace(OLD_SRC, "", 1)
        print("removed about:blank")
    else:
        print("about:blank already absent")
    if OLD_SANDBOX in n:
        n = n.replace(OLD_SANDBOX, NEW_SANDBOX, 1)
        print("sandbox allow-same-origin")
    else:
        print("sandbox already patched or missing")
    if OLD_CSP in n:
        n = n.replace(OLD_CSP, "", 1)
        print("removed iframe csp attribute")
    else:
        print("csp attribute already absent")
    if n == t:
        print("no changes")
        return
    P.write_text(n, encoding="utf-8")
    print("wrote", P, "bytes", P.stat().st_size)


if __name__ == "__main__":
    main()
