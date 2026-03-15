# CodeSys XML Git Diff Driver

Makes `git diff` on CodeSys PLCopen XML exports human-readable by stripping
export noise (timestamps, GUIDs) and normalizing formatting before diffing.

## What it solves

A raw `git diff` on a CodeSys `.xml` export is nearly unreadable — every save
regenerates volatile attributes (`modificationDateTime`, `ObjectGuid`, etc.)
and the attribute order is undefined, producing walls of false changes.

After setup, `git diff` shows only real changes:

```diff
-    IF iCounter >= 100 THEN
+    IF iCounter >= iMaxCount THEN
```

## Files

| File | Purpose |
|------|---------|
| `codesys_xml_normalize.py` | Python script — normalizes XML before diffing |
| `setup_codesys_git_diff.bat` | One-time Windows setup (writes to `~/.gitconfig`) |
| `.gitattributes.template` | Copy to your repo root as `.gitattributes` |

## Quick Start (Windows)

### 1. Copy the three files somewhere permanent

Pick a stable folder, e.g. `C:\Tools\codesys-git-diff\`.
The `setup` script stores that path in `~/.gitconfig`, so **don't move the files
after setup**.

### 2. Run the setup script (once per machine)

Double-click `setup_codesys_git_diff.bat`, or from a terminal:

```bat
setup_codesys_git_diff.bat           # writes to ~/.gitconfig (all repos)
setup_codesys_git_diff.bat --repo    # writes to .git/config (current repo only)
```

### 3. Add `.gitattributes` to each repo

Copy `.gitattributes.template` to your repository root and rename it to
`.gitattributes`, then commit it:

```bat
copy .gitattributes.template .gitattributes
git add .gitattributes
git commit -m "Add CodeSys XML diff driver"
```

> **Tip:** If your repo contains both CodeSys XML *and* other XML files, be
> more specific with the pattern in `.gitattributes`, e.g.:
> ```
> src/plc/*.xml  diff=codesys-xml
> ```

### 4. Verify

```bat
git diff HEAD~1 HEAD -- MyExport.xml
```

You should see only real logic/variable changes, with no timestamp or GUID noise.

---

## How it works

Git's **`textconv`** mechanism transforms a file before diffing it.  The setup
script registers:

```ini
[diff "codesys-xml"]
    textconv      = python "C:/Tools/codesys-git-diff/codesys_xml_normalize.py"
    cachetextconv = true
```

`cachetextconv = true` means git caches the normalized output per blob hash,
so repeated `git log -p` runs are fast.

The normalizer (`codesys_xml_normalize.py`) does the following:

- **Strips noise attributes** — `modificationDateTime`, `DateOfLastChange`,
  `ObjectGuid`, `checkSum`, `buildNumber`, etc.
- **Sorts attributes** — XML has no defined attribute order; sorting gives a
  stable serialization.
- **Consistent indentation** — 2-space indent, one element per line.
- **Preserves all content** — variable declarations, ST/FBD/LD code,
  task configurations, data types — nothing semantic is removed.
- **Encoding tolerant** — handles UTF-8 BOM, UTF-8, and Latin-1 exports.

---

## Customizing noise attributes

Open `codesys_xml_normalize.py` and edit the `NOISE_ATTRIBUTES` set near the
top to add any project-specific volatile attributes:

```python
NOISE_ATTRIBUTES = {
    "DateOfLastChange",
    "modificationDateTime",
    "ObjectGuid",
    # add your own here, e.g.:
    # "MyVendorTimestamp",
}
```

---

## Uninstall

```bat
setup_codesys_git_diff.bat --uninstall           # global
setup_codesys_git_diff.bat --uninstall --repo    # current repo only
```

Then remove the `diff=codesys-xml` lines from `.gitattributes`.

---

## Requirements

- **Python 3.7+** — uses only the standard library (`xml.etree.ElementTree`,
  `xml.dom.minidom`).  No `pip install` needed.
- **Git for Windows** (any recent version).
