#!/usr/bin/env python3
"""PTP clock synchronization status reporter for distributed timing validation.

Parses phc2sys logs or queries the daemon directly to report current
clock offset and synchronization state. Used to verify sub-microsecond
timing accuracy required for multi-sensor data fusion.

Usage:
    python3 ptp_status.py [--phc2sys-log PATH] [--tail LINES] [--phc2sys-bin BIN]

Examples:
    # Check status from default log location
    python3 ptp_status.py

    # Check specific log file
    python3 ptp_status.py --phc2sys-log /var/log/phc2sys.log

    # Query daemon directly (when log unavailable)
    python3 ptp_status.py --phc2sys-bin /usr/sbin/phc2sys

Exit Codes:
    0: Status successfully retrieved and displayed
    2: Failed to read log file
    3: No log found and daemon query failed
    4: Could not parse offset/state from output

Debugging:
    - Verify phc2sys is running: `ps aux | grep phc2sys`
    - Check log file exists: `ls -la /tmp/aeris_phc2sys.log`
    - Run bench_bootstrap.sh to start PTP services

Note:
    Offset values are always displayed in microseconds (µs) regardless
    of the unit in the source log.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from collections import deque
from pathlib import Path
from typing import Deque, Iterable, Optional, Tuple

DEFAULT_LOG = "/tmp/aeris_phc2sys.log"
OFFSET_RE = re.compile(r"offset\s+([-+]?\d+(?:\.\d+)?)\s*(ns|us|ms|s)?", re.IGNORECASE)
STATE_RE = re.compile(r"state\s+([A-Z_]+)", re.IGNORECASE)


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed arguments namespace.
    """
    parser = argparse.ArgumentParser(
        description="Report last-known phc2sys offset",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--phc2sys-log",
        default=DEFAULT_LOG,
        help="Path to phc2sys log (default: %(default)s)",
    )
    parser.add_argument(
        "--tail",
        type=int,
        default=400,
        help="Lines to scan from the log tail (default: %(default)s)",
    )
    parser.add_argument(
        "--phc2sys-bin",
        default="phc2sys",
        help="Binary to call with -m if the log is absent (default: %(default)s)",
    )
    return parser.parse_args()


def read_tail(path: Path, max_lines: int) -> Deque[str]:
    """Read the last N lines from a file.

    Args:
        path: Path to the file to read.
        max_lines: Maximum number of lines to retain.

    Returns:
        Deque containing the last max_lines lines from the file.
    """
    lines: Deque[str] = deque(maxlen=max_lines)
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            lines.append(line.strip())
    return lines


def convert_to_us(value: float, unit: Optional[str]) -> float:
    """Convert a time value to microseconds.

    Args:
        value: The numeric value to convert.
        unit: The unit of the value (ns, us, ms, s) or None for seconds.

    Returns:
        Value converted to microseconds.
    """
    if not unit:
        return value * 1e6  # default seconds when unit not provided
    unit_lc = unit.lower()
    if unit_lc == "ns":
        return value / 1000.0
    if unit_lc == "us":
        return value
    if unit_lc == "ms":
        return value * 1000.0
    if unit_lc == "s":
        return value * 1e6
    return value


def extract_status(lines: Iterable[str]) -> Optional[Tuple[float, Optional[str], str]]:
    """Extract PTP offset and state from log lines.

    Args:
        lines: Iterable of log lines to search.

    Returns:
        Tuple of (offset_us, state, raw_line) or None if no status found.
    """
    for line in reversed(list(lines)):
        offset_match = OFFSET_RE.search(line)
        if not offset_match:
            continue
        offset_us = convert_to_us(float(offset_match.group(1)), offset_match.group(2))
        state_match = STATE_RE.search(line)
        state = state_match.group(1) if state_match else None
        return offset_us, state, line
    return None


def run_phc2sys_once(binary: str) -> Optional[str]:
    """Query phc2sys daemon directly for current status.

    Args:
        binary: Path to the phc2sys binary.

    Returns:
        Output string from the daemon or None if query failed.
    """
    try:
        run = subprocess.run([binary, "-m"], check=False, capture_output=True, text=True, timeout=5)
    except FileNotFoundError:
        return None
    if run.returncode != 0:
        return None
    return run.stdout.strip()


def main() -> int:
    """Main entry point.

    Returns:
        Exit code (0 for success, non-zero for error).
    """
    args = parse_args()
    log_path = Path(args.phc2sys_log)
    lines: Iterable[str]
    source = "log"
    if log_path.exists():
        try:
            lines = read_tail(log_path, args.tail)
        except OSError as exc:
            print(f"Failed to read {log_path}: {exc}", file=sys.stderr)
            return 2
    else:
        snippet = run_phc2sys_once(args.phc2sys_bin)
        if not snippet:
            print(
                f"No phc2sys log at {log_path} and '{args.phc2sys_bin} -m' failed; run bench_bootstrap first?",
                file=sys.stderr,
            )
            return 3
        lines = [snippet]
        source = args.phc2sys_bin + " -m"
    status = extract_status(lines)
    if not status:
        print("Could not locate an offset/state line in phc2sys output.", file=sys.stderr)
        return 4
    offset_us, state, raw_line = status
    state_display = state or "unknown"
    print(f"PTP offset ≈ {offset_us:.3f} µs (state: {state_display}, source: {source})")
    print(f"Last line: {raw_line}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
