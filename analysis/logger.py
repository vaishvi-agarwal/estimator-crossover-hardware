"""Record one session from the ESP32 into data/raw/<session_id>.csv + .json.

Example (Block B, a clean pendulum swing for 60 s):

    python analysis/logger.py --port /dev/cu.usbserial-0001 --block B \
        --string-length-m 0.50 --rest-distance-mm 300 --temp-c 22.5

What it does:
  1. Opens the serial port (921600 baud) and waits for the board to start.
  2. Sends any settings you gave (--cmd "mode frozen", --ge-params file, ...).
  3. Asks the firmware for its settings (`params`), then sends `reset`, which
     restarts the estimators and injector and prints the CSV header.
  4. Records every line for --duration seconds.
  5. Writes the data rows to <session_id>.csv and everything else to a JSON
     "sidecar" next to it: all settings, the measurements you typed in, the
     firmware commit, every '#' line the firmware printed, and row counts.

Anything you do not pass on the command line (temperature, string length,
...) it asks for, so no session is ever saved without its conditions.
Use --no-prompt to skip the questions (fields are then left empty).
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import BLOCKS, DEFAULT_DATA, Paths, git_commit, write_json  # noqa: E402

BAUD = 921600

# Firmware settings each block needs (SPEC.md section 5). Sent before your own
# --cmd commands, so a --cmd can still change them. Use --no-preset to skip.
BLOCK_PRESETS = {
    "A": {"settings": [], "inject": False},                  # static noise
    "B": {"settings": [], "inject": False},                  # clean motion
    "C": {"settings": ["mode variance"], "inject": True},    # variance bursts
    "D": {"settings": ["mode frozen"], "inject": True},      # frozen bursts
    "E": {"settings": [], "inject": False},                  # physical faults: hold the button
    "F": {"settings": [], "inject": True},                   # measured GE parameters (--ge-params)
    "G": {"settings": [], "inject": False},                  # integrated run (esp32dev_guard build)
}


def session_seed(session_id: str) -> int:
    """A different but repeatable injector seed for every session (from its name)."""
    return int(hashlib.sha256(session_id.encode()).hexdigest()[:12], 16)


def build_commands(block: str, session_id: str, user_cmds: list[str], ge_cmd: str | None,
                   preset: bool = True) -> list[str]:
    """The commands sent before recording, in order. logger then adds `params` and `reset`."""
    cmds: list[str] = []
    p = BLOCK_PRESETS[block] if preset else {"settings": [], "inject": None}
    cmds += p["settings"]
    if ge_cmd:
        cmds.append(ge_cmd)
    cmds += user_cmds
    cmds.append(f"seed {session_seed(session_id)}")
    if p["inject"] is not None:
        cmds.append("inject on" if p["inject"] else "inject off")
    return cmds


# ---------------------------------------------------------------------------
# Parsing (kept separate from the serial port so it can be unit-tested)
# ---------------------------------------------------------------------------
def classify_line(line: str) -> str:
    """'meta' for '#' lines, 'header' for the CSV header, 'row' for data, '' for noise."""
    line = line.strip()
    if not line:
        return ""
    if line.startswith("#"):
        return "meta"
    if line.startswith("t_us,"):
        return "header"
    if line[0].isdigit():
        return "row"
    return ""  # boot messages from the ESP32 ROM, partial lines, ...


def parse_meta(line: str) -> tuple[str, dict]:
    """'# params mode=frozen seed=3' -> ('params', {'mode': 'frozen', 'seed': '3'}).

    The first word without '=' is the line's kind (or '' if there is none).
    """
    words = line.lstrip("#").split()
    kind = ""
    fields = {}
    for w in words:
        if "=" in w:
            k, v = w.split("=", 1)
            fields[k] = v
        elif not kind and not fields:
            kind = w
    return kind, fields


def row_ok(line: str, n_cols: int) -> bool:
    """A data row is kept only if it has exactly the header's number of fields."""
    return line.count(",") == n_cols - 1


def summarise(header: list[str], rows: list[str]) -> dict:
    """Row count and lost rows (gaps in the seq column)."""
    out = {"rows": len(rows), "lost_rows": 0}
    if "seq" in header and rows:
        i = header.index("seq")
        seqs = [int(r.split(",")[i]) for r in rows]
        out["lost_rows"] = int(sum(max(0, b - a - 1) for a, b in zip(seqs, seqs[1:])))
    return out


# ---------------------------------------------------------------------------
# Metadata typed in by Vaishvi
# ---------------------------------------------------------------------------
QUESTIONS = [
    # (option name, question, needed for which blocks)
    ("temp_c", "Room temperature (deg C)", "ABCDEFG"),
    ("string_length_m", "Pendulum string length (m)", "BCDEFG"),
    ("rest_distance_mm", "Rest distance, sensor to card (mm)", "BCDEFG"),
    ("amplitude_mm", "Approximate release amplitude (mm)", "BCDEFG"),
    ("card_size_cm", "Card size (cm, e.g. 10x10)", "ABCDEFG"),
    ("target_distance_mm", "Static target distance (mm, tape measure)", "A"),
    ("supply_v", "Sensor supply voltage at VIN (V, multimeter)", "ABCDEFG"),
]


def ask(question: str) -> str:
    try:
        return input(f"{question}: ").strip()
    except EOFError:
        return ""


def collect_user_meta(args) -> dict:
    meta = {}
    for name, question, blocks in QUESTIONS:
        value = getattr(args, name)
        if value is None and args.block in blocks and not args.no_prompt:
            value = ask(question)
        meta[name] = _number_or_text(value)
    notes = args.notes
    if notes is None and not args.no_prompt:
        notes = ask("Notes (what you did, anything unusual)")
    meta["notes"] = notes or ""
    return meta


def _number_or_text(v):
    if v is None or v == "":
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return v


# ---------------------------------------------------------------------------
# Recording
# ---------------------------------------------------------------------------
def ge_command_from_file(path: Path, allow_synthetic: bool) -> str:
    """Block F: the `ge ...` command from the Spec 2 file results/ge_parameters.json."""
    from ge_params import load_ge_parameters  # noqa: PLC0415  (only needed for Block F)

    p = load_ge_parameters(path, allow_synthetic=allow_synthetic)
    return p["command"]


def record(port: str, duration_s: float, commands: list[str]) -> tuple[list[str], list[str], list[dict]]:
    """Send the commands, then record for duration_s. Returns (header, rows, meta_lines)."""
    import serial  # pyserial

    header: list[str] = []
    rows: list[str] = []
    metas: list[dict] = []
    with serial.Serial(port, BAUD, timeout=0.1) as ser:
        time.sleep(2.0)  # opening the port resets the ESP32; let it boot
        ser.reset_input_buffer()
        # `reset` last: restarts the estimators, the injector and seq, and
        # prints the CSV header, so the recording starts from a known state.
        for c in commands + ["params", "reset"]:
            ser.write((c + "\n").encode())
            time.sleep(0.05)
        t_end = time.monotonic() + duration_s
        buf = b""
        while time.monotonic() < t_end or not header:
            if time.monotonic() > t_end + 5:
                raise SystemExit("No CSV header received from the board. Is the right firmware flashed?")
            buf += ser.read(4096)
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                line = raw.decode("ascii", errors="replace").strip()
                kind = classify_line(line)
                if kind == "header":
                    if header and line.split(",") != header:
                        metas.append({"host_t": time.time(), "line": "# warning header changed: " + line})
                    header = line.split(",")
                elif kind == "row" and header and row_ok(line, len(header)):
                    rows.append(line)
                elif kind == "meta":
                    metas.append({"host_t": time.time(), "line": line})
            print(f"\r{len(rows)} rows", end="", flush=True)
    print()
    return header, rows, metas


def save_session(paths: Paths, session_id: str, header: list[str], rows: list[str], metas: list[dict],
                 info: dict) -> tuple[Path, Path]:
    """Write <session_id>.csv and <session_id>.json into <data>/raw."""
    paths.raw.mkdir(parents=True, exist_ok=True)
    csv_path = paths.raw / f"{session_id}.csv"
    json_path = paths.raw / f"{session_id}.json"
    if csv_path.exists() or json_path.exists():
        raise SystemExit(f"{csv_path} already exists; refusing to overwrite raw data.")
    csv_path.write_text(",".join(header) + "\n" + "\n".join(rows) + ("\n" if rows else ""))

    firmware: dict = {}
    events = []
    for m in metas:
        kind, fields = parse_meta(m["line"])
        if kind in ("event", "guard_event"):
            events.append({"kind": kind, **fields})
        else:
            firmware.update(fields)  # later lines (e.g. `params`) override boot lines
    info = dict(info)
    info.update({
        "firmware": firmware,
        "events": events,
        "firmware_lines": [m["line"] for m in metas],
        "counts": summarise(header, rows),
        "columns": header,
    })
    write_json(json_path, info)
    return csv_path, json_path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port, e.g. /dev/cu.usbserial-0001 or COM3")
    ap.add_argument("--block", required=True, choices=sorted(BLOCKS), help="experiment block (SPEC.md section 5)")
    ap.add_argument("--duration", type=float, default=60.0, help="seconds to record (default 60)")
    ap.add_argument("--session-id", help="default: <block>_<date>_<time>")
    ap.add_argument("--data", default=str(DEFAULT_DATA))
    ap.add_argument("--cmd", action="append", default=[],
                    help='firmware command to send first, e.g. --cmd "mode frozen" (repeatable)')
    ap.add_argument("--ge-params", help="Block F: path to results/ge_parameters.json copied from Spec 2")
    ap.add_argument("--no-preset", action="store_true", help="do not send the block's standard settings")
    ap.add_argument("--allow-synthetic-ge", action="store_true", help=argparse.SUPPRESS)
    for name, question, _ in QUESTIONS:
        ap.add_argument("--" + name.replace("_", "-"), dest=name, help=question)
    ap.add_argument("--notes")
    ap.add_argument("--no-prompt", action="store_true", help="do not ask for missing metadata")
    args = ap.parse_args(argv)

    paths = Paths(data=args.data)
    start = dt.datetime.now(dt.timezone.utc)
    session_id = args.session_id or f"{args.block}_{start.astimezone():%Y%m%d_%H%M%S}"
    if args.block == "F" and not args.ge_params:
        raise SystemExit("Block F needs --ge-params results/ge_parameters.json (copied from Spec 2).")
    ge_cmd = ge_command_from_file(Path(args.ge_params), args.allow_synthetic_ge) if args.ge_params else None
    commands = build_commands(args.block, session_id, list(args.cmd), ge_cmd, preset=not args.no_preset)
    user = collect_user_meta(args)

    print(f"Recording {session_id} ({BLOCKS[args.block]}) for {args.duration:.0f} s ...")
    header, rows, metas = record(args.port, args.duration, commands)
    info = {
        "session_id": session_id,
        "block": args.block,
        "block_name": BLOCKS[args.block],
        "start_utc": start.isoformat(),
        "duration_s": args.duration,
        "port": args.port,
        "commands_sent": commands,
        "ge_parameters_file": args.ge_params,
        "user": user,
        "logger_git_commit": git_commit(),
        "synthetic": False,
    }
    csv_path, json_path = save_session(paths, session_id, header, rows, metas, info)
    build = json.loads(json_path.read_text())["firmware"].get("build")
    if (args.block == "G") != (build == "guard"):
        print(f"WARNING: Block {args.block} was recorded with the '{build}' firmware build. "
              "Block G needs `pio run -e esp32dev_guard`, the other blocks `pio run -e esp32dev`.")
    counts = summarise(header, rows)
    print(f"Saved {csv_path} ({counts['rows']} rows, {counts['lost_rows']} lost) and {json_path.name}")
    if counts["lost_rows"]:
        print("WARNING: rows were lost on the serial link. See docs/BRING_UP.md (Milestone 1).")


if __name__ == "__main__":
    main()
