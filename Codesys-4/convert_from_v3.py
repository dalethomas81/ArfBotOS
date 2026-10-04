"""Convert ArfBotOS IEC source from the CODESYS 3 PLCopen export into the
CODESYS 4 file-based project at Codesys-4/ArfBotOS.fbsdev.

Writes programs, function blocks, functions, methods, properties, interfaces,
DUTs, and GVLs. Device tree, EtherCAT, axes, and visualizations are not in
the file format this CODESYS 4 install can store yet, so they are left out.

Re-run from the repo root:
    python Codesys-4/convert_from_v3.py
"""

from __future__ import annotations

import json
import os
import re
import shutil
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML_PATH = os.path.join(REPO, "Codesys", "ArfBot.xml")
# PLCopen XML omits some DUT enumerations (DUT_CommandType and the command
# enums). The native export still has their declaration text.
EXPORT_PATH = os.path.join(REPO, "Codesys", "ArfBot.export")
FBSDEV = os.path.join(REPO, "Codesys-4", "ArfBotOS.fbsdev")
APP_DIR = os.path.join(FBSDEV, "Application.iecapp^")
NS = "{http://www.plcopen.org/xml/tc6_0200}"

# Library programs that this CODESYS 4 install does not provide.
SKIP_TASK_CALLS = {"VisuElems.Visu_Prg"}

# Direct CODESYS 3 library references to recreate when the library is installed.
# (namespace, library title, placeholder name or None)
WANTED_LIBS = [
    ("SysTimeCore", "SysTimeCore", None),
    ("CmpTraceMgr", "CmpTraceMgr", None),
    ("MEMUtils", "MemoryUtils", None),
    ("CAA", "CAA Types Extern", None),
    ("Stu", "StringUtils", "StringUtils"),
    ("SysShm", "SysShm", None),
    ("Util", "Util", "Util"),
    ("SysTypes", "SysTypes Interfaces", None),
    ("SysFile", "SysFile", None),
    ("SysProcess", "SysProcess", None),
    ("CmpBitmapPool", "CmpBitmapPool", None),
    ("SysSocket", "SysSocket", None),
    ("NBS", "Net Base Services", "NetBaseSrv"),
    ("SysTypes2", "SysTypes2 Interfaces", None),
    # SysTimeRtc is installed, but it publishes a global list named GVL.
    # This CODESYS 4 build still imports a direct reference's symbols without
    # a namespace even when allowUnqualifiedAccess is false, so adding the
    # reference makes every GVL_App. use ambiguous. Nothing in the application
    # calls SysTimeRtc. Leave it out; socket and other libraries pull it in
    # privately where it does not leak.
    ("IoDrvGPIO", "IoDrvGPIO", None),
    ("SysCom", "SysCom", None),
    ("CmpDynamicText", "CmpDynamicText", None),
    ("CmpApp", "CmpApp", None),
    ("CmpIecTask", "CmpIecTask", None),
    ("BPLog", "Breakpoint Logging Functions", "BreakpointLogging"),
    ("CmpMgr", "Component Manager", None),
    ("CmpEventMgr", "CmpEventMgr", None),
    ("IoStandard", "IoStandard", None),
    ("DED", "CAA Device Diagnosis", None),
]

# The Raspberry Pi device already publishes this namespace. A second
# application reference makes bootapp compile reject Libraries.lock.json
# ("An item with the same key has already been added. Key: _3S_LICENSE").
DEVICE_NAMESPACES = ["_3S_LICENSE"]

# CODESYS 3 list name. CODESYS 4 libraries (SysSocket Interfaces, and
# SysTimeRtc if referenced directly) also publish a list named GVL into the
# unqualified namespace. The application list is renamed so GVL.X in the
# CODESYS 3 source becomes this name.
GVL_LIST_RENAME = "GVL_App"

# Installed, but the dependency graph is not. Referencing any of these makes
# `c4-cli library install` refuse to write Libraries.lock.json.
# AlarmManager, 3.5.17 (Intern) needs Alarm Manager Interfaces, Alarm Manager
# Internal Interfaces, placeholder CDSV3ProtocolUtils, and Datasources
# Interfaces — none of which ship with this CODESYS 4 install.
UNRESOLVED_LIBS = ["AlarmManager"]

# Referenced by the CODESYS 3 project but not installed with CODESYS 4 yet.
MISSING_LIBS = [
    "SM3_Basic",
    "SM3_Basic_Visu",
    "SM3_CNC",
    "SM3_CNC_Visu",
    "SM3_Robotics",
    "SM3_Robotics_Visu",
    "SM3_Transformation",
    "SM3_Drive_PosControl",
    "OMAC PackML State Machine",
    "IoDrvEtherCAT",
    "VisuElems",
    "VisuDialogs",
    "Visu Utils",
]

OBJECT_TAGS = {
    "pou",
    "Method",
    "Property",
    "Interface",
    "Action",
    "Transition",
    "dataType",
    "union",
    "GetAccessor",
    "SetAccessor",
}
POU_CLOSER = {
    "program": "END_PROGRAM",
    "functionBlock": "END_FUNCTION_BLOCK",
    "function": "END_FUNCTION",
}
POU_EXT = {
    "program": "prg.st",
    "functionBlock": "fb.st",
    "function": "fn.st",
}
HEADER = {
    "program": "PROGRAM",
    "functionBlock": "FUNCTION_BLOCK",
    "function": "FUNCTION",
    "method": "METHOD",
    "property": "PROPERTY",
    "interface": "INTERFACE",
    "dataType": "TYPE",
    "gvl": "VAR_GLOBAL",
    "action": "ACTION",
    "transition": "TRANSITION",
}


def local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def norm(text: str) -> str:
    text = (text or "").replace("\r\n", "\n").replace("\r", "\n")
    if text.startswith("\ufeff"):
        text = text[1:]
    return text.strip("\n")


def element_text(node: ET.Element) -> str:
    """Declaration and body text live in an xhtml child. The surrounding
    XML indentation is a sibling text node and is not part of the source.
    """
    for child in list(node):
        if local(child.tag) == "xhtml":
            return norm("".join(child.itertext()))
    return norm(node.text or "")


def ver_key(version: str):
    parts = []
    for piece in version.split("."):
        try:
            parts.append(int(piece))
        except ValueError:
            parts.append(0)
    return parts


class Converter:
    def __init__(self, root: ET.Element):
        self.root = root
        self.parent = {}
        for parent in root.iter():
            for child in list(parent):
                self.parent[child] = parent
        self.by_id = {}
        self._index_objects()
        self.tasks = self._parse_tasks()
        self.task_names = {task["name"] for task in self.tasks}
        self.written = []
        self.skipped = []
        self.graphical = []
        self.programs = []
        self.missing_decls = []

    def _index_objects(self):
        for elem in self.root.iter():
            tag = local(elem.tag)
            oid = (elem.attrib.get("ObjectId") or "").strip()
            if oid and tag in OBJECT_TAGS:
                self.by_id[oid] = elem
            if tag == "ObjectId" and (elem.text or "").strip():
                owner = elem
                while owner is not None and not self._is_boundary(owner):
                    owner = self.parent.get(owner)
                if owner is not None:
                    self.by_id[elem.text.strip()] = owner

    def _is_boundary(self, elem: ET.Element) -> bool:
        tag = local(elem.tag)
        if tag in OBJECT_TAGS:
            return True
        if tag != "globalVars":
            return False
        cur = self.parent.get(elem)
        while cur is not None:
            if local(cur.tag) in {"pou", "Method", "Property", "Interface", "Action", "Transition"}:
                return False
            cur = self.parent.get(cur)
        return True

    def _closest_boundary(self, node: ET.Element):
        cur = node
        while cur is not None:
            if self._is_boundary(cur):
                return cur
            cur = self.parent.get(cur)
        return None

    def _own_plaintexts(self, elem: ET.Element):
        texts = []
        for node in elem.iter():
            if local(node.tag) != "InterfaceAsPlainText":
                continue
            if self._closest_boundary(node) is elem:
                text = element_text(node)
                if text:
                    texts.append(text)
        return texts

    def _best_decl(self, elem: ET.Element, kind: str) -> str:
        texts = self._own_plaintexts(elem)
        header = HEADER.get(kind, "")
        if kind == "function":
            matched = [t for t in texts if re.search(r"(?m)^FUNCTION(?!_BLOCK)\b", t)]
        elif header:
            matched = [t for t in texts if header in t]
        else:
            matched = texts
        pool = matched or texts
        if not pool:
            return ""
        return max(pool, key=len)

    def _own_body(self, elem: ET.Element):
        for node in elem.iter():
            if local(node.tag) != "body":
                continue
            if self._closest_boundary(node) is not elem:
                continue
            for child in list(node):
                lang = local(child.tag)
                if lang in {"ST", "LD", "FBD", "SFC", "CFC", "IL"}:
                    return lang, element_text(child)
            return "ST", element_text(node)
        return None, ""

    def _parse_tasks(self):
        tasks = []
        for elem in self.root.iter():
            if local(elem.tag) != "task":
                continue
            settings = None
            for node in elem.iter():
                if local(node.tag) == "TaskSettings":
                    settings = node
                    break
            interval = 0
            if settings is not None:
                raw = settings.attrib.get("Interval") or "0"
                unit = settings.attrib.get("IntervalUnit") or "ms"
                try:
                    value = int(raw)
                except ValueError:
                    value = 0
                if unit in {"us", "µs", "μs"}:
                    interval = value
                elif unit == "s":
                    interval = value * 1_000_000
                else:
                    interval = value * 1000
            calls = []
            for child in list(elem):
                if local(child.tag) == "pouInstance":
                    calls.append(child.attrib.get("name") or "")
            try:
                priority = int(elem.attrib.get("priority") or "1")
            except ValueError:
                priority = 1
            tasks.append(
                {
                    "name": elem.attrib.get("name") or "Task",
                    "interval": interval,
                    "priority": priority,
                    "calls": [name for name in calls if name],
                }
            )
        return tasks

    def _kind(self, elem: ET.Element) -> str:
        tag = local(elem.tag)
        if tag == "pou":
            return elem.attrib.get("pouType") or "program"
        if tag == "Method":
            return "method"
        if tag == "Property":
            return "property"
        if tag == "Interface":
            return "interface"
        if tag == "Action":
            return "action"
        if tag == "Transition":
            return "transition"
        if tag == "globalVars":
            return "gvl"
        if tag in {"dataType", "union"}:
            return "dataType"
        return tag

    def _extension(self, elem: ET.Element, kind: str, in_interface: bool) -> str:
        if kind in POU_EXT:
            return POU_EXT[kind]
        if kind == "method":
            return "meth" if in_interface else "meth.st"
        if kind == "property":
            return "prop" if in_interface else "prop.st"
        if kind == "interface":
            return "itf"
        if kind == "action":
            return "act.st"
        if kind == "transition":
            return "trans.st"
        if kind == "gvl":
            return "gvl"
        if local(elem.tag) == "union":
            return "union"
        if kind == "dataType":
            base = None
            for child in elem.iter():
                if local(child.tag) == "baseType":
                    kids = [local(c.tag) for c in list(child)]
                    base = kids[0] if kids else "alias"
                    break
            return {"struct": "struct", "enum": "enum", "union": "union"}.get(base or "", "alias")
        return "st"

    def _accessor(self, prop: ET.Element, name: str):
        for child in prop.iter():
            if local(child.tag) != name:
                continue
            if self._closest_boundary(child) not in {prop, child} and child is not prop:
                # GetAccessor is itself a boundary, so closest of a descendant is the accessor.
                pass
            if local(child.tag) == name and self.parent.get(child) is not None:
                # Only the accessor element that belongs to this property.
                owner = child
                while owner is not None and owner is not prop:
                    owner = self.parent.get(owner)
                if owner is not prop:
                    continue
            decl = self._best_decl(child, "method")
            # Accessor plaintext is not a METHOD. Take the longest text owned by the accessor.
            texts = []
            for node in child.iter():
                if local(node.tag) != "InterfaceAsPlainText":
                    continue
                if self._closest_boundary(node) is child:
                    text = element_text(node)
                    if text:
                        texts.append(text)
            decl = max(texts, key=len) if texts else ""
            lang, body = self._own_body(child)
            if lang and lang != "ST":
                self.graphical.append(f"{prop.attrib.get('name')}.{name} ({lang})")
                body = f"(* CODESYS 3 {name} accessor is {lang} and was not converted. *)"
            if not norm(body) and not _meaningful_accessor_decl(decl):
                return None
            return _compose(decl, body, closer=None)
        return None

    def _compose_object(self, elem: ET.Element, kind: str, label: str, in_interface: bool) -> str:
        decl = self._best_decl(elem, kind)
        lang, body = self._own_body(elem)
        if lang and lang != "ST":
            self.graphical.append(f"{label} ({lang})")
            body = f"(* CODESYS 3 implementation is {lang} and was not converted. *)"
        closer = None
        if kind in POU_CLOSER:
            closer = POU_CLOSER[kind]
        elif kind == "method" and not in_interface:
            closer = "END_METHOD"
        elif kind == "action":
            closer = "END_ACTION"
        elif kind == "transition":
            closer = "END_TRANSITION"
        if not decl and not body:
            self.missing_decls.append(label)
            return ""
        return _compose(decl, body, closer)

    def _emit(self, node, dest: str, in_interface: bool = False):
        if node["kind"] == "folder":
            folder = os.path.join(dest, node["name"])
            os.makedirs(folder, exist_ok=True)
            for child in node["children"]:
                self._emit(child, folder, in_interface)
            return

        name = node["name"]
        if name in self.task_names or name == "Library Manager":
            return
        elem = self.by_id.get(node["id"] or "")
        if elem is None:
            self.skipped.append(f"{name} ({node['id']}) — not in the PLCopen export")
            return

        kind = self._kind(elem)
        ext = self._extension(elem, kind, in_interface)
        # SysSocket publishes SysSocket Interfaces, which also owns a GVL
        # named GVL. See GVL_LIST_RENAME.
        if kind == "gvl" and name == "GVL":
            name = GVL_LIST_RENAME
        filename = f"{name}.{ext}"
        child_interface = in_interface or kind == "interface"

        accessors = []
        if kind == "property":
            for accessor_name in ("GetAccessor", "SetAccessor"):
                text = self._accessor(elem, accessor_name)
                if text:
                    short = "Get" if accessor_name == "GetAccessor" else "Set"
                    accessors.append((f"{short}.meth.st", text))

        emitted_children = [
            child
            for child in node["children"]
            if not (child["kind"] == "object" and child["name"] in self.task_names)
        ]
        has_children = bool(emitted_children or accessors)
        if has_children:
            folder = os.path.join(dest, filename + "^")
            os.makedirs(folder, exist_ok=True)
            target = os.path.join(folder, filename)
            child_dest = folder
        else:
            target = os.path.join(dest, filename)
            child_dest = None

        if kind == "property":
            decl = self._best_decl(elem, "property")
            text = (decl + "\n") if decl else ""
        else:
            text = self._compose_object(elem, kind, name, in_interface)

        if not text.strip():
            self.skipped.append(f"{name} — empty declaration")
            return

        text = _rename_gvl_qualifier(text)
        _write(target, text)
        self.written.append(os.path.relpath(target, APP_DIR))
        if kind == "program":
            self.programs.append(name)

        for accessor_name, accessor_text in accessors:
            path = os.path.join(child_dest, accessor_name)
            _write(path, _rename_gvl_qualifier(accessor_text))
            self.written.append(os.path.relpath(path, APP_DIR))

        if child_dest is not None:
            for child in emitted_children:
                self._emit(child, child_dest, child_interface)

    def convert(self):
        _clean_generated()
        application = _find_application(self.root)
        if application is None:
            raise SystemExit("Application node not found in project structure")
        for child in application["children"]:
            self._emit(child, APP_DIR, False)
        self.supplemented = supplement_from_native_export(_written_names(self.written))
        self._write_tasks()
        self._write_project_info()
        self._patch_bus_cycle()
        added, unavailable, unresolved = write_available_libraries()
        return added, unavailable, unresolved

    def _write_tasks(self):
        task_map = {}
        dropped = []
        for task in self.tasks:
            calls = []
            for name in task["calls"]:
                if name in SKIP_TASK_CALLS:
                    dropped.append(f"{task['name']}: {name}")
                    continue
                if "." in name or name in self.programs:
                    calls.append(name)
                else:
                    dropped.append(f"{task['name']}: {name} (program was not exported)")
            task_map[task["name"]] = {
                "calls": calls,
                "priority": task["priority"],
                "type": "Cyclic",
                "interval": task["interval"],
                "watchdogEnable": False,
                "watchdogTime": 0,
                "watchdogSensitivity": 0,
            }
        payload = {
            "taskGroups": [
                {
                    "groupName": "IEC-Tasks",
                    "tasks": task_map,
                    "coreConfig": "FixedPinned",
                }
            ],
            "systemEvents": [],
        }
        path = os.path.join(APP_DIR, "TaskConfiguration.json")
        _write_json(path, payload)
        self.dropped_calls = dropped

    def _write_project_info(self):
        path = os.path.join(FBSDEV, "ProjectInfo.json")
        _write_json(
            path,
            {
                "title": "ArfBotOS",
                "version": "1.0.0",
                "author": "Dale Thomas",
                "company": "Thomas Labs",
                "description": "https://github.com/dalethomas81/ArfBotOS",
                "defaultNamespace": "ArfBotOS",
            },
        )

    def _patch_bus_cycle(self):
        path = os.path.join(
            FBSDEV, "Devices", "CODESYS_Control_for_Raspberry_Pi_64_SL.device.json"
        )
        if not os.path.isfile(path):
            return
        text = open(path, encoding="utf-8").read()
        text2 = text.replace('"busCycleTask": "DefaultTask"', '"busCycleTask": "EtherCAT_Task"')
        if text2 != text:
            _write(path, text2 if text2.endswith("\n") else text2 + "\n")


def _meaningful_accessor_decl(decl: str) -> bool:
    compact = re.sub(r"\s+", "", decl or "")
    return compact not in {"", "VAREND_VAR"}


def _compose(decl: str, body: str, closer: str | None) -> str:
    parts = []
    decl = norm(decl)
    body = norm(body)
    if decl:
        parts.append(decl)
    if body:
        if parts:
            parts.append("")
        parts.append(body)
    if closer:
        tail = (parts[-1] if parts else "").rstrip()
        if not tail.endswith(closer):
            parts.append(closer)
    return "\n".join(parts) + "\n"


def _rename_gvl_qualifier(text: str) -> str:
    return text.replace("GVL.", GVL_LIST_RENAME + ".")


def _write(path: str, text: str):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)


def _write_json(path: str, payload):
    text = json.dumps(payload, indent="\t", ensure_ascii=False) + "\n"
    _write(path, text)


def _clean_generated():
    plc = os.path.join(APP_DIR, "PLC_PRG.prg.st")
    if os.path.isfile(plc):
        os.remove(plc)
    for name in os.listdir(APP_DIR):
        path = os.path.join(APP_DIR, name)
        if name.startswith("."):
            continue
        if name in {"Libraries.json", "Libraries.lock.json", "TaskConfiguration.json"}:
            continue
        if os.path.isdir(path):
            shutil.rmtree(path)
        elif name.endswith((".st", ".gvl", ".struct", ".enum", ".alias", ".union", ".itf", ".meth", ".prop")):
            os.remove(path)


def _written_names(relative_paths):
    names = set()
    for rel in relative_paths:
        base = os.path.basename(rel)
        for ext in (
            ".prg.st",
            ".fb.st",
            ".fn.st",
            ".meth.st",
            ".act.st",
            ".trans.st",
            ".prop.st",
            ".struct",
            ".enum",
            ".alias",
            ".union",
            ".gvl",
            ".itf",
            ".meth",
            ".prop",
        ):
            if base.endswith(ext):
                names.add(base[: -len(ext)])
                break
    return names


def _leading_type_name(blob: str) -> str | None:
    for line in blob.split("\n"):
        stripped = line.strip()
        if not stripped or stripped.startswith("//") or stripped.startswith("{attribute"):
            continue
        match = re.match(r"TYPE\s+([A-Za-z0-9_]+)\b", stripped)
        return match.group(1) if match else None
    return None


def _dut_extension(blob: str) -> str:
    if re.search(r"(?m)^UNION\b", blob):
        return "union"
    if re.search(r"(?m)^STRUCT\b", blob):
        return "struct"
    if re.search(r"(?m)^\(", blob):
        return "enum"
    return "alias"


def supplement_from_native_export(already: set[str]) -> list[str]:
    """Write DUT declarations that the PLCopen export does not contain."""
    if not os.path.isfile(EXPORT_PATH):
        return []
    text = open(EXPORT_PATH, encoding="utf-8", errors="replace").read()
    name_re = re.compile(r'<Single Name="Name" Type="string">([^<]+)</Single>')
    matches = list(name_re.finditer(text))
    added = []
    seen = set(already)
    for index, match in enumerate(matches):
        name = match.group(1)
        if name in seen:
            continue
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        chunk = text[match.end() : end]
        blob_m = re.search(
            r'<Single Name="TextBlobForSerialisation" Type="string">(.*?)</Single>',
            chunk,
            re.DOTALL,
        )
        if blob_m is None:
            continue
        blob = norm(blob_m.group(1))
        type_name = _leading_type_name(blob)
        if type_name != name:
            continue
        path_m = re.search(r'<Array Name="Path" Type="string">(.*?)</Array>', chunk, re.DOTALL)
        parts = []
        if path_m is not None:
            parts = re.findall(r"<Single Type=\"string\">([^<]*)</Single>", path_m.group(1))
        if "Application" in parts:
            rel = parts[parts.index("Application") + 1 :]
        else:
            rel = ["DUT"]
        dest = os.path.join(APP_DIR, *rel, f"{name}.{_dut_extension(blob)}")
        _write(dest, _rename_gvl_qualifier(blob) + "\n")
        added.append(os.path.relpath(dest, APP_DIR))
        seen.add(name)
    return added


def _find_application(root: ET.Element):
    for elem in root.iter():
        if local(elem.tag) != "ProjectStructure":
            continue
        return _walk_for_application(_node_from_xml(elem))
    return None


def _node_from_xml(elem: ET.Element):
    children = []
    for child in list(elem):
        tag = local(child.tag)
        if tag in {"Folder", "Object"}:
            children.append(_node_from_xml(child))
    tag = local(elem.tag)
    if tag == "Folder":
        return {"kind": "folder", "name": elem.attrib.get("Name") or "", "id": "", "children": children}
    return {
        "kind": "object",
        "name": elem.attrib.get("Name") or "",
        "id": (elem.attrib.get("ObjectId") or "").strip(),
        "children": children,
    }


def _walk_for_application(node):
    if node["kind"] == "object" and node["name"] == "Application":
        return node
    for child in node["children"]:
        found = _walk_for_application(child)
        if found is not None:
            return found
    return None


def discover_installed_libs():
    roots = [os.path.join(r"C:\Program Files\CODESYS-4", "extensions")]
    found = {}
    for root in roots:
        if not os.path.isdir(root):
            continue
        for dirpath, _dirs, files in os.walk(root):
            if not any("compiled-library" in name for name in files):
                continue
            if os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(dirpath)))) != "c4Libs":
                continue
            version = os.path.basename(dirpath)
            title = os.path.basename(os.path.dirname(dirpath))
            company = os.path.basename(os.path.dirname(os.path.dirname(dirpath)))
            found.setdefault(title, []).append((company, version))
    return found


def write_available_libraries():
    installed = discover_installed_libs()
    path = os.path.join(APP_DIR, "Libraries.json")
    data = json.loads(open(path, encoding="utf-8").read())
    refs = data.setdefault("references", {})
    added = []
    unavailable = []
    unresolved = []
    for namespace in UNRESOLVED_LIBS:
        if namespace in refs:
            del refs[namespace]
        unresolved.append(namespace)
    for namespace in DEVICE_NAMESPACES:
        refs.pop(namespace, None)
    for namespace, title, placeholder in WANTED_LIBS:
        if namespace in refs:
            continue
        if placeholder and title in installed:
            refs[namespace] = {"$type": "Placeholder", "placeholder": placeholder}
            added.append(f"{namespace} -> placeholder {placeholder}")
            continue
        options = installed.get(title) or []
        if not options:
            unavailable.append(f"{namespace} ({title})")
            continue
        company, version = max(options, key=lambda item: ver_key(item[1]))
        refs[namespace] = {
            "$type": "ManagedLibrary",
            "libraryId": f"{title}, {version} ({company})",
        }
        added.append(f"{namespace} -> {title}, {version} ({company})")
    data["placeholderOverrides"] = data.get("placeholderOverrides") or {}
    _write_json(path, data)
    return added, unavailable, unresolved


def main():
    root = ET.parse(XML_PATH).getroot()
    converter = Converter(root)
    added, unavailable, unresolved = converter.convert()
    print(f"Wrote {len(converter.written)} IEC files")
    print(f"Programs: {', '.join(converter.programs)}")
    print(f"Tasks: {', '.join(task['name'] for task in converter.tasks)}")
    if converter.dropped_calls:
        print("Task calls left out:")
        for item in converter.dropped_calls:
            print(f"  {item}")
    if converter.graphical:
        print("Graphical implementations not converted:")
        for item in converter.graphical:
            print(f"  {item}")
    if converter.missing_decls:
        print("Missing declarations:")
        for item in converter.missing_decls:
            print(f"  {item}")
    if converter.skipped:
        print("Skipped objects:")
        for item in converter.skipped:
            print(f"  {item}")
    if converter.supplemented:
        print(f"Types recovered from the native export: {len(converter.supplemented)}")
        for item in converter.supplemented:
            print(f"  {item}")
    print(f"Libraries added: {len(added)}")
    for item in added:
        print(f"  {item}")
    if unresolved:
        print("Installed, but left out because dependencies are missing:")
        for item in unresolved:
            print(f"  {item}")
    if unavailable:
        print("Libraries from CODESYS 3 that are not installed here:")
        for item in unavailable:
            print(f"  {item}")
    print("Not installed in CODESYS 4 (motion, fieldbus, visualization):")
    for item in MISSING_LIBS:
        print(f"  {item}")


if __name__ == "__main__":
    main()
