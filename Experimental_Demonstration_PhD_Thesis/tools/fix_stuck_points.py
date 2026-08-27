#!/usr/bin/env python3
"""
fix_stuck_points.py - Detect and remove "stuck robot" points from sub3_spatial datasets.

Background
----------
sub3_spatial.cpp moves the UR5 to each target of positions3D.txt, records a
{K}-orientation voltage scan and then marks that target as done (last column
0 -> 1). If the UR5 enters a protective stop (e.g. it touches itself) the
Python server still answers "reachable" and still returns a pose, so the
campaign keeps running and the SAME physical location is measured over and
over. Nothing is written to run.log, and pose_px/py/pz keep changing because
they are the COMMANDED pose - so the only trace left in the data is that the
voltage vector stops changing between consecutive points.

A second, unrelated defect lives in the same files: if a run is interrupted
and later resumed with 'C', the point that was in progress is measured AGAIN
under the SAME point_id, so its rows contain a truncated/garbage first attempt
glued in front of the good one. Those stale attempts are detected too.

What this tool does
-------------------
1. Groups master.csv rows by point_id and splits each point into measurement
   "attempts" (a new attempt starts at every vertical scan of orientation 1).
   Only the LAST attempt of a point is real data; earlier ones are leftovers
   from an interrupted run.
2. Builds a per-point "fingerprint": the vector of v_mean of the vertical scan
   of the last attempt, ordered by orientation_id.
3. Finds maximal runs of points whose fingerprint equals that of the point
   RIGHT BEFORE it within a tolerance (default 3 mV) -> the robot did not move
   between them. The comparison is step-to-step on purpose: comparing every
   point against the first one of the run would break a genuinely frozen block
   as soon as the photodiode's slow thermal drift exceeded the tolerance.
4. Prints everything (fingerprints included, so they can be eyeballed exactly
   like the MATLAB table) and asks for confirmation before:
     - rewriting master.csv / kp1.csv without the stuck rows and the stale
       attempt rows,
     - resetting the matching lines of positions3D.txt back to done=0
       (only for the stuck points: a re-measured point stays done).

Backups (<name>.bak_YYYYmmdd_HHMMSS) are written before any file is touched.

Usage
-----
  # report only (never writes anything)
  python tools/fix_stuck_points.py output/sub3_spatial/20260728_110846 --dry-run

  # interactive fix of the three affected sessions at once
  python tools/fix_stuck_points.py ^
      output/sub3_spatial/20260728_110846 ^
      output/sub3_spatial/20260729_105752 ^
      output/sub3_spatial/20260730_111944

  # keep the first point of every run (see --keep-first below)
  python tools/fix_stuck_points.py <session> --keep-first
"""

import argparse
import os
import shutil
import sys
from datetime import datetime

# Coordinates in master.csv / positions3D.txt are grid values (0.2 m steps),
# so an exact-ish comparison is safe.
COORD_TOL = 1e-6

# How many fingerprint values to show per row when printing the tables.
MAX_SHOWN = 12


# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------
def to_float(text):
    """Parse a CSV field into a float, returning None for 'NA'/empty/garbage."""
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def read_text_lines(path):
    """Read a text file as a list of lines (no newlines) + trailing-newline flag."""
    with open(path, "r", encoding="utf-8-sig", newline="") as fh:
        text = fh.read()
    ends_with_newline = text.endswith("\n")
    lines = text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    return lines, ends_with_newline


def write_text_lines(path, lines, ends_with_newline=True):
    """Write lines back with Unix newlines (what the C++ writer also produces)."""
    with open(path, "w", encoding="utf-8", newline="") as fh:
        fh.write("\n".join(lines))
        if ends_with_newline:
            fh.write("\n")


def backup(path, stamp):
    """Copy path to path.bak_<stamp>. Returns the backup path."""
    dst = "{}.bak_{}".format(path, stamp)
    shutil.copy2(path, dst)
    return dst


def confirm(question, assume_yes=False):
    """Ask a yes/no question. Returns True only on an explicit yes."""
    if assume_yes:
        print("{} [auto-yes]".format(question))
        return True
    try:
        answer = input("{} [y/N]: ".format(question)).strip().lower()
    except EOFError:
        return False
    return answer in ("y", "yes")


# ---------------------------------------------------------------------------
# master.csv parsing
# ---------------------------------------------------------------------------
class Point(object):
    """One point_id of a session: its rows, attempts and voltage fingerprint."""

    __slots__ = ("pid", "x", "y", "z", "line_idx", "attempts",
                 "fingerprint", "fp_source")

    def __init__(self, pid):
        self.pid = pid
        self.x = self.y = self.z = None
        self.line_idx = []       # every data-line index, in file order
        self.attempts = []       # list of lists of data-line indices
        self.fingerprint = None  # tuple of floats of the LAST attempt
        self.fp_source = ""

    @property
    def coord(self):
        return (self.x, self.y, self.z)

    def coord_str(self):
        return "({:g}, {:g}, {:g})".format(self.x, self.y, self.z)

    @property
    def stale_lines(self):
        """Data-line indices of every attempt except the last (valid) one."""
        out = []
        for attempt in self.attempts[:-1]:
            out.extend(attempt)
        return out


def split_attempts(rows):
    """Split one point's rows into measurement attempts.

    Per point sub3_spatial writes one 'vertical' scan (orientation 1..K) and
    then the tilt scans. If the campaign is interrupted and resumed with 'C',
    the point that was in progress is measured again under the same point_id,
    so its rows hold several attempts glued together. A new attempt therefore
    starts at every 'vertical' row with repeat_id == 1 and orientation_id == 1.

    `rows` is a list of (line_idx, scan_kind, repeat, orient, v_mean).
    Returns a list of lists of positions into `rows`.
    """
    starts = [k for k, r in enumerate(rows)
              if r[1] == "vertical" and r[2] == 1.0 and r[3] == 1.0]
    if not starts:
        return [list(range(len(rows)))]
    if starts[0] != 0:
        starts.insert(0, 0)
    blocks = []
    for a, s in enumerate(starts):
        end = starts[a + 1] if a + 1 < len(starts) else len(rows)
        blocks.append(list(range(s, end)))
    return blocks


def build_fingerprint(attempt_rows):
    """Voltage vector of one attempt -> (tuple_or_None, source_label)."""
    if not attempt_rows:
        return None, ""

    chosen = [r for r in attempt_rows if r[1] == "vertical"]
    source = "vertical"
    if not chosen:
        # Session without a vertical scan: take the first scan block, i.e. rows
        # up to the point where the orientation index restarts.
        seen = set()
        for r in attempt_rows:
            if r[3] in seen:
                break
            seen.add(r[3])
            chosen.append(r)
        source = attempt_rows[0][1]
    if not chosen:
        return None, source

    chosen = sorted(chosen, key=lambda r: (r[2], r[3]))
    values = [r[4] for r in chosen]
    if any(v is None for v in values):
        return None, source
    return tuple(values), source


def parse_master(path):
    """Parse master.csv -> (header_line, data_lines, points_in_order)."""
    lines, _ = read_text_lines(path)
    if not lines:
        raise ValueError("{}: file is empty".format(path))

    header_line = lines[0]
    header = header_line.split(",")
    col = {name: i for i, name in enumerate(header)}
    for required in ("point_id", "x", "y", "z", "scan_kind", "orientation_id",
                     "repeat_id", "v_mean"):
        if required not in col:
            raise ValueError("{}: missing column '{}'".format(path, required))

    data_lines = [ln for ln in lines[1:] if ln.strip()]

    points = []
    by_pid = {}
    meta = {}   # pid -> list of (line_idx, scan_kind, repeat, orient, v_mean)

    for i, line in enumerate(data_lines):
        f = line.split(",")
        if len(f) < len(header):
            continue
        pid = f[col["point_id"]]

        point = by_pid.get(pid)
        if point is None:
            point = Point(pid)
            point.x = to_float(f[col["x"]])
            point.y = to_float(f[col["y"]])
            point.z = to_float(f[col["z"]])
            by_pid[pid] = point
            points.append(point)
            meta[pid] = []
        point.line_idx.append(i)
        meta[pid].append((i,
                          f[col["scan_kind"]],
                          to_float(f[col["repeat_id"]]) or 0.0,
                          to_float(f[col["orientation_id"]]) or 0.0,
                          to_float(f[col["v_mean"]])))

    for point in points:
        rows = meta[point.pid]
        blocks = split_attempts(rows)
        point.attempts = [[rows[k][0] for k in blk] for blk in blocks]
        point.fingerprint, point.fp_source = build_fingerprint(
            [rows[k] for k in blocks[-1]])

    return header_line, data_lines, points


# ---------------------------------------------------------------------------
# stuck-run detection
# ---------------------------------------------------------------------------
def max_abs_diff(a, b):
    """Max |a_i - b_i|, or None when the vectors are not comparable."""
    if a is None or b is None or len(a) != len(b):
        return None
    return max(abs(x - y) for x, y in zip(a, b))


def find_runs(points, tol, min_run):
    """Find maximal runs of consecutive points whose fingerprint never changes.

    Each point is compared with its IMMEDIATE PREDECESSOR - "the robot did not
    move between these two points" - and NOT with the run anchor. Comparing
    against the anchor splits a genuinely frozen block as soon as the slow
    thermal drift of the photodiode exceeds the tolerance (~2 mV over 7 h is
    enough), which invents boundaries in the middle of the block and makes
    --keep-first silently keep invalid points.
    """
    runs = []
    i = 0
    n = len(points)
    while i < n:
        if points[i].fingerprint is None:
            i += 1
            continue
        j = i + 1
        while j < n:
            diff = max_abs_diff(points[j - 1].fingerprint, points[j].fingerprint)
            if diff is None or diff > tol:
                break
            j += 1
        if (j - i) >= min_run:
            runs.append(list(range(i, j)))
            i = j
        else:
            i += 1
    return runs


def fmt_fingerprint(fp):
    if fp is None:
        return "<unusable>"
    shown = ["{:8.4f}".format(v) for v in fp[:MAX_SHOWN]]
    tail = " ..." if len(fp) > MAX_SHOWN else ""
    return " ".join(shown) + tail


def fmt_step(value):
    """Format a step-to-step deviation, tolerating incomparable vectors."""
    if value is None:
        return "  step=  n/a  (different vector length)"
    return "  step={:.5f} V".format(value)


def describe_run(points, run, tol):
    """Human-readable block for one detected run."""
    out = []
    anchor = points[run[0]]
    last = points[run[-1]]
    out.append("  Run of {} consecutive points with a frozen voltage vector"
               " (scan '{}', tol {:.4f} V):".format(len(run), anchor.fp_source, tol))

    drift = max_abs_diff(anchor.fingerprint, last.fingerprint)
    if drift is not None:
        out.append("    total drift first -> last: {:.5f} V  (slow thermal drift,"
                   " not movement)".format(drift))

    before = run[0] - 1
    if before >= 0:
        prev = points[before]
        diff = max_abs_diff(prev.fingerprint, anchor.fingerprint)
        out.append("    - previous point {:<28} {:<22} {}".format(
            prev.pid, prev.coord_str(), fmt_fingerprint(prev.fingerprint)))
        if diff is not None:
            out.append("      step into the run: {:.5f} V (> tol) -> the robot really"
                       " moved between these two points".format(diff))

    for pos, k in enumerate(run):
        point = points[k]
        if pos == 0:
            step_txt = "  step=  --     (run start)"
        else:
            step_txt = fmt_step(max_abs_diff(points[run[pos - 1]].fingerprint,
                                             point.fingerprint))
        out.append("    * {:<30} {:<22} {}{}".format(
            point.pid, point.coord_str(), fmt_fingerprint(point.fingerprint), step_txt))

    after = run[-1] + 1
    if after < len(points):
        nxt = points[after]
        diff = max_abs_diff(last.fingerprint, nxt.fingerprint)
        out.append("    - next point     {:<28} {:<22} {}".format(
            nxt.pid, nxt.coord_str(), fmt_fingerprint(nxt.fingerprint)))
        if diff is not None:
            out.append("      step out of the run: {:.5f} V (> tol) -> movement"
                       " resumed here".format(diff))
    else:
        out.append("    - next point     <none: the run reaches the end of the session>")
    return out


# ---------------------------------------------------------------------------
# fingerprint dump (for plotting in MATLAB)
# ---------------------------------------------------------------------------
def dump_fingerprints(path, points):
    width = max((len(p.fingerprint) for p in points if p.fingerprint), default=0)
    header = ["point_id", "x", "y", "z", "scan"]
    header += ["v{}".format(i + 1) for i in range(width)]
    rows = [",".join(header)]
    for p in points:
        cells = [p.pid, "{:g}".format(p.x), "{:g}".format(p.y), "{:g}".format(p.z),
                 p.fp_source or "NA"]
        fp = p.fingerprint or ()
        cells += ["{:.8g}".format(v) for v in fp]
        cells += ["NA"] * (width - len(fp))
        rows.append(",".join(cells))
    write_text_lines(path, rows)


# ---------------------------------------------------------------------------
# positions3D.txt repair
# ---------------------------------------------------------------------------
def reset_positions(path, coords, stamp, dry_run):
    """Set the 'done' column back to 0 for every coordinate in `coords`.

    Only the last token of a matching line is rewritten, so the original number
    formatting produced by savePositions() is preserved byte for byte.
    """
    lines, ends_nl = read_text_lines(path)
    targets = list(coords)
    matched = {}
    new_lines = list(lines)
    changed = 0
    already_zero = 0

    for i, line in enumerate(lines):
        tok = line.split()
        if len(tok) < 4:
            continue
        x, y, z = to_float(tok[0]), to_float(tok[1]), to_float(tok[2])
        if x is None or y is None or z is None:
            continue
        for c in targets:
            if (abs(x - c[0]) <= COORD_TOL and abs(y - c[1]) <= COORD_TOL
                    and abs(z - c[2]) <= COORD_TOL):
                matched.setdefault(c, []).append(i + 1)
                if tok[3] == "0":
                    already_zero += 1
                else:
                    new_lines[i] = " ".join(tok[:3] + ["0"])
                    changed += 1
                break

    missing = [c for c in targets if c not in matched]
    duplicated = {c: v for c, v in matched.items() if len(v) > 1}

    print("\n=== positions3D.txt ===")
    print("  File            : {}".format(path))
    print("  Coordinates to reset : {}".format(len(targets)))
    print("  Lines set 1 -> 0     : {}".format(changed))
    print("  Already 0 (no-op)    : {}".format(already_zero))
    if duplicated:
        print("  [WARN] coordinates matching more than one line:")
        for c, v in duplicated.items():
            print("         ({:g}, {:g}, {:g}) -> lines {}".format(c[0], c[1], c[2], v))
    if missing:
        print("  [WARN] coordinates NOT found in the positions file:")
        for c in missing:
            print("         ({:g}, {:g}, {:g})".format(c[0], c[1], c[2]))

    if changed == 0:
        print("  Nothing to change.")
        return 0
    if dry_run:
        print("  [dry-run] positions file left untouched.")
        return 0

    print("  Backup          : {}".format(backup(path, stamp)))
    write_text_lines(path, new_lines, ends_nl)
    print("  positions3D.txt updated: {} point(s) can be re-measured.".format(changed))
    return changed


# ---------------------------------------------------------------------------
# per-session processing
# ---------------------------------------------------------------------------
def resolve_master(target):
    """Accept either a session directory or a master.csv path."""
    if os.path.isdir(target):
        return os.path.join(target, "master.csv")
    return target


def process_session(master_path, args, stamp):
    """Analyse one session. Returns the list of coordinates to reset."""
    session_dir = os.path.dirname(os.path.abspath(master_path))
    print("\n" + "=" * 78)
    print(" SESSION: {}".format(session_dir))
    print("=" * 78)

    if not os.path.isfile(master_path):
        print("  [ERROR] not found: {}".format(master_path))
        return []

    header_line, data_lines, points = parse_master(master_path)
    usable = sum(1 for p in points if p.fingerprint is not None)
    print("  master.csv  : {} data rows, {} points ({} with a usable fingerprint)"
          .format(len(data_lines), len(points), usable))

    if args.dump_fingerprints:
        fp_path = os.path.join(session_dir, "fingerprints.csv")
        dump_fingerprints(fp_path, points)
        print("  fingerprints: {}".format(fp_path))

    report = []
    report.append("Session   : {}".format(session_dir))
    report.append("Generated : {}".format(datetime.now().isoformat(timespec="seconds")))
    report.append("Tolerance : {} V   min run: {}   keep-first: {}   keep-stale: {}"
                  .format(args.tol, args.min_run, args.keep_first, args.keep_stale))
    report.append("")

    # --- finding 1: point_ids measured more than once (interrupted + resumed)
    stale_idx = set()
    dup_points = [p for p in points if len(p.attempts) > 1]
    if dup_points:
        block = ["  --- Points measured more than once (interrupted + resumed) ---"]
        for p in dup_points:
            sizes = [len(a) for a in p.attempts]
            block.append("    {:<30} {:<20} attempts={}  rows={}  -> keep last ({} rows)"
                         .format(p.pid, p.coord_str(), len(p.attempts), sizes, sizes[-1]))
        if args.keep_stale:
            block.append("    -> --keep-stale given: these rows are left untouched.")
        else:
            for p in dup_points:
                stale_idx.update(p.stale_lines)
            block.append("    -> {} stale row(s) from the earlier attempt(s) queued"
                         " for deletion.".format(len(stale_idx)))
        print("")
        for line in block:
            print(line)
        report.extend(block)
        report.append("")
    else:
        print("  Duplicated point_id : none")

    # --- finding 2: frozen-voltage runs (the stuck robot) -------------------
    runs = find_runs(points, args.tol, args.min_run)
    doomed = []   # point objects to remove entirely
    if not runs:
        print("  Frozen-voltage runs : none")
        report.append("No frozen-voltage run detected.")
        report.append("")
    else:
        for n, run in enumerate(runs, 1):
            block = describe_run(points, run, args.tol)
            print("\n  --- Detected run {}/{} ---".format(n, len(runs)))
            for line in block:
                print(line)
            report.append("--- Run {}/{} ---".format(n, len(runs)))
            report.extend(block)

            victims = run[1:] if args.keep_first else run
            if args.keep_first:
                note = ("    -> keeping {} (first of the run), removing {} point(s)."
                        .format(points[run[0]].pid, len(victims)))
            else:
                note = "    -> removing all {} point(s) of the run.".format(len(victims))
            print(note)
            report.append(note)
            report.append("")
            doomed.extend(points[k] for k in victims)

    doomed_pids = set(p.pid for p in doomed)
    drop_idx = set(stale_idx)
    for p in doomed:
        drop_idx.update(p.line_idx)

    if not drop_idx:
        print("\n  Nothing to clean in this session.")
        return []

    print("\n  === SUMMARY for this session ===")
    print("  Frozen runs detected    : {}".format(len(runs)))
    print("  Stuck points to remove  : {}".format(len(doomed)))
    print("  Stale attempt rows      : {}".format(len(stale_idx)))
    print("  master.csv rows deleted : {} of {}".format(len(drop_idx), len(data_lines)))
    if runs and not args.keep_first:
        print("  NOTE: the first point of each run is also removed, because it is")
        print("        impossible to prove the robot had reached its target before")
        print("        freezing. Use --keep-first if you want to trust it.")

    if doomed:
        report.append("Stuck points removed ({}):".format(len(doomed)))
        for p in doomed:
            report.append("  {}  {}".format(p.pid, p.coord_str()))
        report.append("")

    # Only the stuck points have to be re-measured. A point that was correctly
    # re-measured after a resume keeps its valid data, so it stays done=1.
    coords = [p.coord for p in doomed]

    if args.dry_run:
        print("\n  [dry-run] master.csv left untouched.")
        return coords

    if not confirm("\n  Delete these {} rows from master.csv?".format(len(drop_idx)),
                   args.yes):
        print("  Skipped: master.csv not modified, positions not queued.")
        return []

    # --- rewrite master.csv -------------------------------------------------
    kept = [ln for i, ln in enumerate(data_lines) if i not in drop_idx]
    print("  Backup      : {}".format(backup(master_path, stamp)))
    write_text_lines(master_path, [header_line] + kept)
    print("  master.csv  : {} rows removed, {} rows kept.".format(len(drop_idx), len(kept)))

    # --- rewrite kp1.csv (cooperative measurements), if it has rows ---------
    kp1_path = os.path.join(session_dir, "kp1.csv")
    if os.path.isfile(kp1_path):
        kp1_lines, _ = read_text_lines(kp1_path)
        if len(kp1_lines) > 1:
            kp1_header = kp1_lines[0]
            kp1_data = [ln for ln in kp1_lines[1:] if ln.strip()]
            kp1_kept = [ln for ln in kp1_data
                        if ln.split(",")[0] not in doomed_pids]
            removed = len(kp1_data) - len(kp1_kept)
            if removed:
                print("  Backup      : {}".format(backup(kp1_path, stamp)))
                write_text_lines(kp1_path, [kp1_header] + kp1_kept)
                print("  kp1.csv     : {} rows removed.".format(removed))

    # --- report file --------------------------------------------------------
    report_path = os.path.join(session_dir, "stuck_report_{}.txt".format(stamp))
    write_text_lines(report_path, report)
    print("  Report      : {}".format(report_path))

    return coords


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------
def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(here)
    default_positions = os.path.join(project_root, "src", "positionsToSample",
                                     "positions3D.txt")

    parser = argparse.ArgumentParser(
        description="Detect and remove stuck-robot points from sub3_spatial sessions.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sessions", nargs="+",
                        help="session directory (or master.csv path); repeatable")
    parser.add_argument("--positions", default=default_positions,
                        help="positions3D.txt to repair (default: %(default)s)")
    parser.add_argument("--tol", type=float, default=0.010,
                        help="max |dV| between two consecutive fingerprints for the "
                             "robot to count as not having moved, in volts "
                             "(default: %(default)s = 10 mV). Measured on the "
                             "20260728/29/30 sessions: consecutive points at the SAME "
                             "frozen location differ by up to ~5 mV, while the "
                             "smallest genuine move is ~40 mV, and the result is "
                             "identical for any tolerance between 6 and 30 mV.")
    parser.add_argument("--min-run", type=int, default=2, dest="min_run",
                        help="minimum number of consecutive identical points to flag "
                             "(default: %(default)s)")
    parser.add_argument("--keep-first", action="store_true", dest="keep_first",
                        help="keep the first point of each run (assume the robot did "
                             "reach it before freezing)")
    parser.add_argument("--keep-stale", action="store_true", dest="keep_stale",
                        help="do NOT delete the rows of earlier attempts of a point "
                             "that was re-measured after a resume")
    parser.add_argument("--dump-fingerprints", action="store_true",
                        dest="dump_fingerprints",
                        help="also write <session>/fingerprints.csv for MATLAB plots")
    parser.add_argument("--dry-run", action="store_true", dest="dry_run",
                        help="only report; never modify any file")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompts (non-interactive)")
    args = parser.parse_args(argv)

    if args.min_run < 2:
        parser.error("--min-run must be >= 2 (a single point is never 'frozen')")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    print("fix_stuck_points.py - frozen-voltage detector for sub3_spatial")
    print("  tolerance : {} V".format(args.tol))
    print("  min run   : {} consecutive points".format(args.min_run))
    print("  mode      : {}".format("DRY-RUN (read only)" if args.dry_run else "interactive"))

    all_coords = []
    for target in args.sessions:
        master_path = resolve_master(target)
        try:
            all_coords.extend(process_session(master_path, args, stamp))
        except (ValueError, OSError) as exc:
            print("  [ERROR] {}: {}".format(master_path, exc))

    if not all_coords:
        print("\nNothing to reset in positions3D.txt. Done.")
        return 0

    # De-duplicate coordinates while keeping a stable order.
    unique = []
    seen = set()
    for c in all_coords:
        if c not in seen:
            seen.add(c)
            unique.append(c)

    print("\n" + "=" * 78)
    print(" POSITIONS FILE REPAIR")
    print("=" * 78)
    print("  {} removed point(s) must go back to done=0 so the campaign"
          " re-measures them:".format(len(unique)))
    for c in unique:
        print("    ({:g}, {:g}, {:g})".format(c[0], c[1], c[2]))

    if not os.path.isfile(args.positions):
        print("  [ERROR] positions file not found: {}".format(args.positions))
        return 1

    if args.dry_run:
        reset_positions(args.positions, unique, stamp, dry_run=True)
        print("\n[dry-run] No file was modified.")
        return 0

    if not confirm("\n  Reset these {} position(s) to done=0 in {}?"
                   .format(len(unique), os.path.basename(args.positions)), args.yes):
        print("  Skipped: positions3D.txt not modified.")
        print("  WARNING: master.csv rows were already deleted, so those points are")
        print("           now missing from the dataset AND still flagged as done.")
        return 1

    reset_positions(args.positions, unique, stamp, dry_run=False)
    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
