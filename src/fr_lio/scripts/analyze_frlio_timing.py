#!/usr/bin/env python3
"""Apply the FR-LIO prop-off timing acceptance limits to FRLIO_TIMING logs."""

import argparse
from datetime import datetime
import json
import re
from pathlib import Path


NUMBER = re.compile(r"([A-Za-z_]+)=([^\s]+)")
IMU_STAGE = "imu_callback_arrival"
LIDAR_STAGE = "lidar_frame_arrival"
HIGH_RATE_STAGE = "high_rate_odom_publish"
TIMESTAMP_STAGES = (IMU_STAGE, LIDAR_STAGE)
ANCHOR_ACCEPT_MIN_S = 0.30
ANCHOR_ACCEPT_MAX_S = 0.50
IMU_QUEUE_RUNAWAY = 100
HIGH_RATE_CATASTROPHIC_GAP_US = 1_000_000


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    return values[int(round((len(values) - 1) * fraction))]


def values(records, key):
    return [float(record[key]) for record in records if key in record]


def stat(records, key):
    data = values(records, key)
    return {
        "count": len(data),
        "p95": percentile(data, 0.95),
        "max": max(data) if data else None,
    }


def read_records(path):
    records = []
    for line in path.read_text(errors="replace").splitlines():
        if "FRLIO_TIMING" not in line:
            continue
        record = dict(NUMBER.findall(line))
        if record.get("stage"):
            records.append(record)
    return records


def event_timestamp(text):
    # GNU date emits nanoseconds after a comma, while Python 3.10 accepts at
    # most microseconds and expects a dot.
    normalized = re.sub(
        r"([,.]\d{6})\d+(?=[+-]\d{2}:\d{2}$)", r"\1", text
    ).replace(",", ".")
    return datetime.fromisoformat(normalized).timestamp()


def excluded_intervals(events_path, labels):
    if not labels:
        return []
    events = {}
    for line in events_path.read_text(encoding="utf-8").splitlines():
        try:
            stamp, label = line.split("\t", 1)
            events[label] = event_timestamp(stamp)
        except ValueError as exc:
            raise ValueError(f"invalid event row: {line!r}") from exc

    intervals = []
    for begin, end in labels:
        if begin not in events or end not in events:
            raise ValueError(f"missing event marker(s): {begin}, {end}")
        if events[end] < events[begin]:
            raise ValueError(f"event interval is reversed: {begin}, {end}")
        intervals.append({"begin": begin, "end": end,
                          "start_s": events[begin], "stop_s": events[end]})
    return intervals


def filter_excluded(records, intervals):
    if not intervals:
        return records, 0
    kept = []
    excluded = 0
    for record in records:
        timestamp = float(record.get("sensor_timestamp", "nan"))
        if any(interval["start_s"] <= timestamp <= interval["stop_s"]
               for interval in intervals):
            excluded += 1
        else:
            kept.append(record)
    return kept, excluded


def timestamp_rollbacks(records):
    rollbacks = []
    sample_counts = {}
    for stage in TIMESTAMP_STAGES:
        previous = None
        count = 0
        for record in records:
            if record.get("stage") != stage or "sensor_timestamp" not in record:
                continue
            timestamp = float(record["sensor_timestamp"])
            if previous is not None and timestamp <= previous:
                rollbacks.append({"stage": stage, "from": previous, "to": timestamp})
            previous = timestamp
            count += 1
        sample_counts[stage] = count
    return sample_counts, rollbacks


def longest_queue_run(records, threshold):
    longest = 0
    current = 0
    for record in records:
        queue_size = int(float(record["imu_queue"]))
        if queue_size >= threshold:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="FR-LIO log containing FRLIO_TIMING lines")
    parser.add_argument("--json", dest="json_path", type=Path)
    parser.add_argument("--high-rate-hz", type=int, choices=(30, 50), default=30)
    parser.add_argument("--events", type=Path,
                        help="event_wall_time.tsv used with --exclude-event-interval")
    parser.add_argument("--exclude-event-interval", action="append", nargs=2,
                        metavar=("BEGIN", "END"), default=[])
    args = parser.parse_args()
    if args.exclude_event_interval and args.events is None:
        parser.error("--exclude-event-interval requires --events")

    records = read_records(args.log)
    intervals = excluded_intervals(args.events, args.exclude_event_interval)
    records, excluded_count = filter_excluded(records, intervals)
    imu_records = [record for record in records if record.get("stage") == IMU_STAGE]
    high_rate_records = [
        record for record in records if record.get("stage") == HIGH_RATE_STAGE
    ]
    gap_limit_us = 100_000 if args.high_rate_hz == 30 else 80_000

    stats = {
        "mutex_hold_us": stat(records, "mutex_hold_us"),
        "imu_queue": stat(imu_records, "imu_queue"),
        "high_rate_gap_us": stat(high_rate_records, "gap_us"),
        "predictor_state_age_us": stat(imu_records, "predictor_state_age_us"),
        "anchor_age_s": stat(records, "anchor_age"),
    }
    mutex = stats["mutex_hold_us"]
    queue = stats["imu_queue"]
    gap = stats["high_rate_gap_us"]
    age = stats["predictor_state_age_us"]
    sample_counts, rollbacks = timestamp_rollbacks(records)
    queue_run = longest_queue_run(imu_records, IMU_QUEUE_RUNAWAY)

    anchor_couplings = []
    for record in records:
        if "anchor_age" not in record:
            continue
        anchor_age = float(record["anchor_age"])
        if not ANCHOR_ACCEPT_MIN_S <= anchor_age <= ANCHOR_ACCEPT_MAX_S:
            continue
        queue_runaway = int(float(record.get("imu_queue", "0"))) >= IMU_QUEUE_RUNAWAY
        catastrophic_gap = (
            record.get("stage") == HIGH_RATE_STAGE
            and float(record.get("gap_us", "0")) > HIGH_RATE_CATASTROPHIC_GAP_US
        )
        if queue_runaway or catastrophic_gap:
            anchor_couplings.append({
                "stage": record.get("stage"),
                "anchor_age_s": anchor_age,
                "imu_queue": int(float(record.get("imu_queue", "0"))),
                "high_rate_gap_us": float(record.get("gap_us", "0")),
            })

    checks = {
        "mutex_hold_p95_lt_1ms": mutex["p95"] is not None and mutex["p95"] < 1_000,
        "mutex_hold_max_lt_5ms": mutex["max"] is not None and mutex["max"] < 5_000,
        "imu_queue_max_lt_30": queue["max"] is not None and queue["max"] < 30,
        "imu_queue_no_sustained_100_plus": queue_run == 0,
        "high_rate_gap_max_under_rate_limit": gap["max"] is not None and gap["max"] < gap_limit_us,
        "predictor_state_age_p95_lt_50ms": age["p95"] is not None and age["p95"] < 50_000,
        "predictor_state_age_max_lt_100ms": age["max"] is not None and age["max"] < 100_000,
        "sensor_timestamp_samples_present": all(
            sample_counts[stage] >= 2 for stage in TIMESTAMP_STAGES
        ),
        "sensor_timestamp_strictly_monotonic": not rollbacks,
        "anchor_age_300_500ms_not_coupled_to_runaway_or_1s_gap": not anchor_couplings,
    }
    result = {
        "log": str(args.log),
        "high_rate_hz": args.high_rate_hz,
        "record_count": len(records),
        "excluded_record_count": excluded_count,
        "excluded_intervals": intervals,
        "stage_record_counts": {
            IMU_STAGE: len(imu_records),
            HIGH_RATE_STAGE: len(high_rate_records),
            LIDAR_STAGE: sum(record.get("stage") == LIDAR_STAGE for record in records),
        },
        "stats": stats,
        "limits": {
            "mutex_hold_p95_us": 1_000,
            "mutex_hold_max_us": 5_000,
            "imu_queue_max": 30,
            "imu_queue_runaway": IMU_QUEUE_RUNAWAY,
            "high_rate_gap_us": gap_limit_us,
            "predictor_state_age_p95_us": 50_000,
            "predictor_state_age_max_us": 100_000,
            "anchor_age_accepted_range_s": [ANCHOR_ACCEPT_MIN_S, ANCHOR_ACCEPT_MAX_S],
            "anchor_age_disallowed_companions": {
                "imu_queue": IMU_QUEUE_RUNAWAY,
                "high_rate_gap_us": HIGH_RATE_CATASTROPHIC_GAP_US,
            },
        },
        "imu_queue_100_plus_longest_run": queue_run,
        "timestamp_sample_counts": sample_counts,
        "timestamp_rollbacks": rollbacks[:20],
        "timestamp_rollback_count": len(rollbacks),
        "anchor_age_300_500ms_couplings": anchor_couplings[:20],
        "anchor_age_300_500ms_coupling_count": len(anchor_couplings),
        "checks": checks,
        "passed": bool(records) and all(checks.values()),
        "note": (
            "anchor_age in 300-500 ms is diagnostic-only unless it coincides with "
            "imu_queue >= 100 or a high-rate output gap > 1 s."
        ),
    }
    output = json.dumps(result, ensure_ascii=False, indent=2)
    print(output)
    if args.json_path:
        args.json_path.write_text(output + "\n", encoding="utf-8")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
