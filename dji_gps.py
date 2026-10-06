#!/usr/bin/env python3
"""Extract timestamped DJI camera-pose candidates as JSON."""

import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys

PROTOCOL = "dvtm_Mini5Pro.proto"
VERSION = "02.01.07"
POSITION_TAGS = ("GPSLatitude", "GPSLongitude", "AbsoluteAltitude")
QUATERNION_TAGS = tuple(f"Dvtm_Mini5Pro_3-4-4-{i}" for i in range(1, 5))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def transpose(matrix):
    return list(zip(*matrix))


def multiply(a, b):
    return [[dot(row, column) for column in transpose(b)] for row in a]


def geodetic(position):
    """WGS84 ECEF position and ECEF-to-local-ENU rotation."""
    latitude, longitude = map(math.radians, position[:2])
    slat, clat = math.sin(latitude), math.cos(latitude)
    slon, clon = math.sin(longitude), math.cos(longitude)
    radius = 6378137 / math.sqrt(1 - 6.69437999014e-3 * slat * slat)
    height = position[2]
    point = [(radius + height) * clat * clon, (radius + height) * clat * slon,
             (radius * (1 - 6.69437999014e-3) + height) * slat]
    basis = [[-slon, clon, 0], [-slat * clon, -slat * slon, clat],
             [clat * clon, clat * slon, slat]]
    return point, basis


def pose(position, quaternion, origin):
    """Provisional ENU-to-camera transform; navigation heading is uncalibrated."""
    norm = math.sqrt(dot(quaternion, quaternion))
    w, x, y, z = (value / norm for value in quaternion)
    attitude = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    point, local_basis = geodetic(position)
    reference, basis = geodetic(origin)
    delta = [a-b for a, b in zip(point, reference)]
    center = [dot(row, delta) for row in basis]
    rotation = multiply(basis, transpose(local_basis))
    for factor in ([[0, 1, 0], [1, 0, 0], [0, 0, -1]], attitude,
                   [[0, 0, 1], [1, 0, 0], [0, 1, 0]]):
        rotation = multiply(rotation, factor)
    return [list(row) + [-dot(row, center)] for row in transpose(rotation)] + [[0, 0, 0, 1]]


def decode(tags):
    """Decode the verified schema; never guess orientation fields for another model."""
    documents = {}
    for key, value in tags.items():
        group, _, tag = key.partition(":")
        if re.fullmatch(r"Doc\d+", group):
            documents.setdefault(group, {})[tag] = value
    if not documents:
        raise ValueError("no embedded telemetry; use the original DJI video")
    samples = list(documents.values())
    if ({s["Protocol"] for s in samples if "Protocol" in s} != {PROTOCOL}
            or {s["Dvtm_Mini5Pro_1-1-2"] for s in samples if "Dvtm_Mini5Pro_1-1-2" in s} != {VERSION}):
        raise ValueError("unsupported orientation schema; expected Mini5Pro 02.01.07")
    frames = []
    for group, sample in documents.items():
        try:
            time = float(sample["SampleTime"])
            position = [float(sample[tag]) for tag in POSITION_TAGS]
            # Protobuf omits zero components. An entirely absent quaternion
            # still fails validation; no identity attitude is invented.
            quaternion = [float(re.search(r"float ([^\s)]+)",
                          str(sample.get(tag, "float 0")))[1]) for tag in QUATERNION_TAGS]
            if (not all(map(math.isfinite, [time, *position, *quaternion]))
                    or not (-90 <= position[0] <= 90 and -180 <= position[1] <= 180)
                    or abs(math.sqrt(dot(quaternion, quaternion)) - 1) > .001):
                raise ValueError("invalid position or quaternion")
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            raise ValueError(f"invalid or incomplete pose sample {group}: {error}") from error
        frames.append(dict(time_s=time, position_wgs84=position,
                           gimbal_quaternion_wxyz=quaternion))
    frames.sort(key=lambda frame: frame["time_s"])
    if any(a["time_s"] >= b["time_s"] for a, b in zip(frames, frames[1:])):
        raise ValueError("duplicate telemetry timestamps")
    origin = frames[0]["position_wgs84"]
    for frame in frames:
        frame["world_to_camera"] = pose(frame["position_wgs84"], frame["gimbal_quaternion_wxyz"], origin)
    return dict(
        format="dji-video-poses-v1", schema=PROTOCOL, schema_version=VERSION,
        world=dict(axes="east,north,up", units="metres", origin_wgs84=origin,
                   altitude_datum="unverified; reported AbsoluteAltitude used directly"),
        camera_axes="right,down,forward", transform="world_to_camera",
        quaternion_convention="wxyz; gimbal forward-right-down to DJI navigation frame",
        heading_reference="unverified; transform assumes navigation heading equals geographic heading",
        timing="video SampleTime in seconds; exposure latency uncalibrated",
        position_reference="reported GPS point; GPS-to-camera lever arm uncalibrated",
        validity="telemetry-derived pose candidates; not image-refined or calibrated",
        frames=frames,
    )


def extract(video: Path):
    if not video.is_file():
        raise ValueError(f"video not found: {video}")
    result = subprocess.run(
        ["exiftool", "-ee", "-u", "-G3", "-s", "-n", "-j", str(video.resolve())],
        check=True, capture_output=True, text=True,
    )
    return decode(json.loads(result.stdout)[0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    args = parser.parse_args()
    try:
        result = extract(args.video)
    except FileNotFoundError:
        parser.exit(1, "error: install ExifTool and put exiftool on PATH\n")
    except subprocess.CalledProcessError as error:
        parser.exit(1, f"error: ExifTool failed: {error.stderr.strip()}\n")
    except (ValueError, TypeError, KeyError, IndexError) as error:
        parser.exit(1, f"error: {error}\n")
    json.dump(result, sys.stdout, indent=2, allow_nan=False)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
