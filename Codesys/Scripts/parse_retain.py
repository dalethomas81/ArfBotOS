"""Parse ArfBotOS BackupRetain.ret and generate M_BuildTests ST.

Usage:
    python parse_retain.py [path-to-BackupRetain.ret]
"""
from pathlib import Path
import struct
import sys

DEFAULT_RETAIN = Path(r"C:\Users\dalet\OneDrive\Desktop\Original\BackupRetain.ret")
ST_OUT = Path(__file__).resolve().parent / "st" / "M_BuildTests_impl.st"

HEADER = 68
PROG_SIZE = 17082
N_PROG = 20
N_CMD = 50
CMD_SIZE = 340
NAME_LEN = 81
CMD0_OFF = 82
TYPE_SIZE = 2
STR_LEN = 256
CMT_LEN = 81

STORED_ALIGN = 4  # pad after Programs so StoredPositions is 8-byte aligned
POS_SIZE = 136
POS_COMMENT = 81
POS_PAD = 7
N_POS = 50

CMD_TYPES = {
    0: "DUT_CommandType.Disabled",
    1: "DUT_CommandType.MoveCommand",
    2: "DUT_CommandType.PauseCommand",
    3: "DUT_CommandType.InputCommand",
    4: "DUT_CommandType.OutputCommand",
    5: "DUT_CommandType.VisionCommand",
    6: "DUT_CommandType.CncCommand",
    7: "DUT_CommandType.VarSetCommand",
    8: "DUT_CommandType.LogicCommand",
}


def cstr(b):
    z = b.find(b"\x00")
    if z < 0:
        z = len(b)
    return b[:z].decode("latin-1", errors="replace")


def st_str(s):
    # Doubled quotes ('') are ambiguous when a STRING ends with an
    # apostrophe — CODESYS eats the closer. Use $' / $$ instead.
    return "'" + s.replace("$", "$$").replace("'", "$'") + "'"


def parse(data):
    programs = []
    for pi in range(N_PROG):
        base = HEADER + pi * PROG_SIZE
        name = cstr(data[base : base + NAME_LEN])
        cmds = []
        for ci in range(N_CMD):
            cbase = base + CMD0_OFF + ci * CMD_SIZE
            ctype = struct.unpack_from("<H", data, cbase)[0]
            cstrng = cstr(data[cbase + TYPE_SIZE : cbase + TYPE_SIZE + STR_LEN])
            ccmt = cstr(
                data[cbase + TYPE_SIZE + STR_LEN : cbase + TYPE_SIZE + STR_LEN + CMT_LEN]
            )
            cmds.append({"type": ctype, "string": cstrng, "comment": ccmt})
        programs.append({"name": name, "commands": cmds})

    pos_start = HEADER + N_PROG * PROG_SIZE + STORED_ALIGN
    positions = []
    for i in range(N_POS):
        pbase = pos_start + i * POS_SIZE
        comment = cstr(data[pbase : pbase + POS_COMMENT])
        xyz = struct.unpack_from("<6d", data, pbase + POS_COMMENT + POS_PAD)
        positions.append({"comment": comment, "xyzabc": xyz})
    return programs, positions


def fmt_lreal(v):
    s = "{0:.6f}".format(v).rstrip("0").rstrip(".")
    if "." not in s:
        s += ".0"
    return s


def emit_stored_position(lines, index, pos, gvl_name=None):
    target = (
        "PersistentVars.StoredPositions[{0}]".format(gvl_name)
        if gvl_name
        else "PersistentVars.StoredPositions[{0}]".format(index)
    )
    if pos["comment"]:
        lines.append("{0}.Comment := {1};".format(target, st_str(pos["comment"])))
    for axis, val in zip("XYZABC", pos["xyzabc"]):
        lines.append("{0}.Position.{1} := {2};".format(target, axis, fmt_lreal(val)))


def generate_st(programs, positions):
    lines = []
    # Taught positions the factory programs depend on. PC1 is currently 0,0,0
    # so it is not written (M_Init / default zeros already cover that).
    lines.append("// taught positions referenced by factory programs")
    emit_stored_position(lines, 40, positions[40])
    emit_stored_position(lines, 41, positions[41])
    emit_stored_position(lines, 42, positions[42])
    lines.append("")

    for pi, prog in enumerate(programs):
        active = [
            (ci, cmd)
            for ci, cmd in enumerate(prog["commands"])
            if cmd["type"] != 0
        ]
        if not prog["name"] and not active:
            continue
        # skip unnamed leftover slots with little content
        if not prog["name"] and len(active) <= 1:
            continue

        lines.append("//")
        lines.append("FOR _i := GVL.PROG_CMDS_ARR_BEG TO GVL.PROG_CMDS_ARR_END BY 1 DO")
        lines.append("	PersistentVars.Programs[{0}].Commands[_i].CommandType := DUT_CommandType.Disabled;".format(pi))
        lines.append("	PersistentVars.Programs[{0}].Commands[_i].CommandString := '';".format(pi))
        lines.append("	PersistentVars.Programs[{0}].Commands[_i].CommandComment := '';".format(pi))
        lines.append("END_FOR")
        lines.append("PersistentVars.Programs[{0}].ProgramName := {1};".format(pi, st_str(prog["name"])))
        for ci, cmd in active:
            tname = CMD_TYPES.get(cmd["type"])
            if tname is None:
                raise RuntimeError("unknown command type {0} at program {1} cmd {2}".format(cmd["type"], pi, ci))
            prefix = "PersistentVars.Programs[{0}].Commands[{1}]".format(pi, ci)
            lines.append("{0}.CommandType := {1};".format(prefix, tname))
            lines.append("{0}.CommandString := {1};".format(prefix, st_str(cmd["string"])))
            if cmd["comment"]:
                lines.append("{0}.CommandComment := {1};".format(prefix, st_str(cmd["comment"])))
        lines.append("")
    text = "\n".join(lines).rstrip() + "\n"
    return text


def main():
    retain = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_RETAIN
    data = retain.read_bytes()
    print("retain", retain, "size", len(data))
    programs, positions = parse(data)

    print("=== PROGRAMS ===")
    for pi, prog in enumerate(programs):
        n_active = sum(1 for c in prog["commands"] if c["type"] != 0)
        print("[{0:02d}] {1!r}  active={2}".format(pi, prog["name"], n_active))

    print("\n=== STORED POSITIONS (named or nonzero) ===")
    for i, pos in enumerate(positions):
        x, y, z, a, b, c = pos["xyzabc"]
        if pos["comment"] or any(abs(v) > 1e-12 for v in pos["xyzabc"]):
            print("[{0:02d}] {1!r}  X={2} Y={3} Z={4} A={5} B={6} C={7}".format(
                i, pos["comment"], x, y, z, a, b, c
            ))

    st = generate_st(programs, positions)
    ST_OUT.parent.mkdir(parents=True, exist_ok=True)
    ST_OUT.write_text(st, encoding="utf-8")
    print("\nWrote", ST_OUT, "({0} chars)".format(len(st)))


if __name__ == "__main__":
    main()
