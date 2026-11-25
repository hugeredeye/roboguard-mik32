"""Extract RoboGuard CSV blocks and plot IMU data from a UART log."""

import argparse
import csv
import json
import math
import re
from pathlib import Path


BLOCKS = {
    "EVENT": ("history", "seq", "trigger_hint", 10),
    "FAST RAM": ("fast_ram", "fast_seq", "trigger", 2),
    "FAST FLASH": ("fast_flash", "fast_seq", "trigger", 2),
}
MARKER = re.compile(r"=== (EVENT|FAST RAM|FAST FLASH) CSV (BEGIN|END) ===")
AXES = ("ax", "ay", "az", "gx", "gy", "gz")


def read_blocks(log):
    raw = log.read_bytes()
    encoding = "utf-16" if raw.startswith((b"\xff\xfe", b"\xfe\xff")) else "utf-8-sig"
    kind = None
    for line_number, line in enumerate(raw.decode(encoding, errors="replace").splitlines(), 1):
        line = line.strip()
        marker = MARKER.fullmatch(line)
        if marker:
            name, boundary = marker.groups()
            if boundary == "BEGIN":
                kind, start, metadata, rows, saw_header = name, line_number, {}, [], False
            elif kind == name:
                expected = metadata.get("records", len(rows))
                if not saw_header or not rows or len(rows) != expected:
                    raise ValueError(f"Invalid {kind} CSV starting on line {start}: "
                                     f"{len(rows)} rows, expected {expected}")
                yield kind, start, metadata, rows
                kind = None
            continue
        if kind is None or not line:
            continue
        _, sequence, trigger, _ = BLOCKS[kind]
        if not saw_header:
            saw_header = line == ",".join((sequence, *AXES, trigger))
            if not saw_header:
                metadata.update((key, int(value)) for key, value in re.findall(r"(\w+)=(\d+)", line))
            continue
        try:
            row = tuple(map(int, line.split(",")))
            if len(row) != 8 or row[-1] not in (0, 1):
                raise ValueError
        except ValueError as exc:
            raise ValueError(f"Malformed {kind} CSV row on line {line_number}") from exc
        rows.append(row)
    if kind is not None:
        raise ValueError(f"UART log ends inside the {kind} CSV starting on line {start}")


def plot_window(series, output, name, has_trigger):
    from matplotlib.figure import Figure

    figure = Figure(figsize=(9, 8), layout="constrained")
    plots = figure.subplots(3, 1, sharex=True)
    time_ms, ax, ay, az, gx, gy, gz, accel, gyro = zip(*series)
    for plot, values, labels, units in (
        (plots[0], (ax, ay, az), ("X", "Y", "Z"), "Acceleration (g)"),
        (plots[1], (gx, gy, gz), ("X", "Y", "Z"), "Angular rate (°/s)"),
        (plots[2], (accel,), ("Magnitude",), "Acceleration (g)"),
    ):
        for samples, label in zip(values, labels):
            plot.plot(time_ms, samples, label=label)
        if has_trigger:
            plot.axvline(0, color="0.35", linestyle="--", linewidth=1)
        plot.set_ylabel(units)
        plot.grid(alpha=0.25)
        plot.legend(loc="upper right")
    plots[2].set_xlabel("Nominal time from trigger (ms)" if has_trigger else "Nominal time from first sample (ms)")
    plots[2].twinx().plot(time_ms, gyro, color="darkorange", alpha=0.8)
    figure.axes[-1].set_ylabel("Angular rate magnitude (°/s)", color="darkorange")
    figure.suptitle(name)
    figure.savefig(output, dpi=140)


def export_window(block, output, label, plots):
    kind, line, metadata, rows = block
    name, sequence, trigger, period = BLOCKS[kind]
    basename = f"{label}_{line:05d}_{name}"
    with (output / f"{basename}.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow((sequence, *AXES, trigger))
        writer.writerows(rows)

    trigger_seq = next((row[0] for row in rows if row[-1]), None)
    origin = rows[0][0] if trigger_seq is None else trigger_seq
    series = []
    for seq, ax, ay, az, gx, gy, gz, _ in rows:
        # Firmware configures ±16 g and ±2000 °/s. Sequence gives nominal time.
        accel = (ax / 2048, ay / 2048, az / 2048)
        gyro = (gx / 16.4, gy / 16.4, gz / 16.4)
        relative_seq = (seq - origin + 2**31) % 2**32 - 2**31
        series.append((relative_seq * period, *accel, *gyro, math.hypot(*accel), math.hypot(*gyro)))
    if plots:
        plot_window(series, output / f"{basename}.png", basename, trigger_seq is not None)
    return {
        "file": basename,
        "kind": name,
        "records": len(rows),
        "event_seq": metadata.get("event_seq"),
        "event_type": metadata.get("event_type"),
        "nominal_period_ms": period,
        "trigger_seq": trigger_seq,
        "trigger_is_approximate": trigger_seq is not None and kind == "EVENT",
        "peak_acceleration_g": round(max(sample[-2] for sample in series), 3),
        "peak_angular_rate_dps": round(max(sample[-1] for sample in series), 3),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="UART log from PlatformIO Device Monitor")
    parser.add_argument("--out", type=Path, help="Output folder (default: LOGSTEM_report beside the log)")
    parser.add_argument("--label", help="Run label, for example impact_01 (defaults to log filename)")
    parser.add_argument("--no-plots", action="store_true", help="Export CSV and JSON without matplotlib")
    args = parser.parse_args()
    label = args.label or args.log.stem
    if re.fullmatch(r"[\w.-]+", label) is None:
        parser.error("use letters, digits, underscores, dots or dashes in the run label")
    output = args.out or args.log.with_name(f"{args.log.stem}_report")
    try:
        blocks = list(read_blocks(args.log))
        if not blocks:
            raise ValueError("No complete RoboGuard CSV blocks found")
        output.mkdir(parents=True, exist_ok=True)
        results = [export_window(block, output, label, not args.no_plots) for block in blocks]
        (output / f"{label}_summary.json").write_text(
            json.dumps(results, indent=2), encoding="utf-8"
        )
    except (OSError, ValueError, ImportError) as exc:
        parser.exit(1, f"RoboGuard: {exc}\n")
    print(f"Exported {len(results)} windows to {output}")


if __name__ == "__main__":
    main()
