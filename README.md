# dji-gps: DJI video pose extraction

Extract timestamped camera-pose candidates from embedded DJI video telemetry.
Python 3.9+ and [ExifTool](https://exiftool.org/) only; no Python packages,
API key, flight log, or network access.

```sh
brew install exiftool
python3 dji_gps.py /path/to/DJI_video.MP4 > poses.json
python3 -m unittest
```

The output is **JSON, replacing the original GPS-only CSV**. Each `frames` entry
contains `time_s`, source `position_wgs84` (latitude degrees, longitude degrees,
reported absolute altitude metres), source `gimbal_quaternion_wxyz`, and a 4x4
`world_to_camera` matrix. For a homogeneous column vector, `p_camera =
world_to_camera @ p_world`. World positions use metres in a local east/north/up
frame anchored at the first sample. Camera axes are right/down/forward.

These are **telemetry-derived pose candidates**, with the same conversion as
Splatstudio's DJI decoder, not image-refined camera poses. The quaternion maps
gimbal forward/right/down axes into DJI's navigation frame; it is not a
body-relative gimbal joint angle. Do not add drone heading to it.

The transform provisionally assumes navigation heading is geographic heading.
The navigation-to-ground heading correction, GPS-to-camera lever arm, altitude
datum, and telemetry-to-exposure latency are uncalibrated and explicitly recorded
in the JSON. `time_s` is ExifTool's video sample time, not UTC. Source GPS and
quaternion values are retained; the quaternion is normalized only for the matrix.

Orientation decoding currently supports the verified `dvtm_Mini5Pro.proto`
schema, version `02.01.07`. Other schemas fail explicitly instead of guessing
their axes or fields. Videos must retain embedded telemetry; edited exports may
discard it. Missing/invalid poses and duplicate timestamps are errors.

No image extraction, calibration, pose refinement, reconstruction, or training.
