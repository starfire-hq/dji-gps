#!/usr/bin/env python3
"""Extract timestamped embedded DJI GPS samples as CSV."""

import argparse
import csv
import json
import math
from pathlib import Path
import re
import subprocess
import sys

TAGS = ("SampleTime", "GPSLatitude", "GPSLongitude", "AbsoluteAltitude")
COLUMNS = ("time_s", "latitude_deg", "longitude_deg", "absolute_altitude_m")


def extract(video: Path) -> list[tuple[float, ...]]:
    """Return reported GPS samples, ordered by video sample time."""
    if not video.is_file():
        raise ValueError(f"video not found: {video}")
    result = subprocess.run(
        ["exiftool", "-ee", "-G3", "-s", "-n", "-j",
         *(f"-{tag}" for tag in TAGS), str(video.resolve())],
        check=True, capture_output=True, text=True,
    )
    documents = {}
    for key, value in json.loads(result.stdout)[0].items():
        group, _, tag = key.partition(":")
        if re.fullmatch(r"Doc\d+", group):
            documents.setdefault(group, {})[tag] = value
    rows = []
    for group, sample in documents.items():
        if not any(tag in sample for tag in TAGS[1:]):
            continue
        if any(tag not in sample for tag in TAGS):
            raise ValueError(f"incomplete GPS sample: {group}")
        row = tuple(float(sample[tag]) for tag in TAGS)
        if not all(map(math.isfinite, row)) or not (-90 <= row[1] <= 90 and -180 <= row[2] <= 180):
            raise ValueError(f"invalid GPS sample: {group}")
        rows.append(row)
    if not rows:
        raise ValueError("no embedded DJI GPS samples found; use the original telemetry-bearing video")
    return sorted(rows, key=lambda row: row[0])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    args = parser.parse_args()
    try:
        rows = extract(args.video)
    except FileNotFoundError:
        parser.exit(1, "error: ExifTool is required; install it and put exiftool on PATH\n")
    except subprocess.CalledProcessError as error:
        parser.exit(1, f"error: ExifTool failed: {error.stderr.strip()}\n")
    except (ValueError, TypeError, KeyError, IndexError) as error:
        parser.exit(1, f"error: {error}\n")
    writer = csv.writer(sys.stdout, lineterminator="\n")
    writer.writerow(COLUMNS)
    writer.writerows(rows)


if __name__ == "__main__":
    main()
