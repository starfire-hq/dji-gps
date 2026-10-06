# dji-gps

Extract the embedded, timestamped GPS samples from an original DJI video.
Runs locally with Python 3.9+ and [ExifTool](https://exiftool.org/).
No Python packages, DJI API key, flight log, or network access required.

```sh
brew install exiftool
python3 dji_gps.py /path/to/DJI_video.MP4 > gps.csv
```

CSV columns: `time_s`, `latitude_deg`, `longitude_deg`, `absolute_altitude_m`.
Time is ExifTool's video sample time in seconds, not UTC or calibrated exposure
time. Coordinates are reported GPS latitude/longitude in degrees. Altitude is
DJI's reported `AbsoluteAltitude` in metres; its vertical datum is not verified.
Values are exported without smoothing, interpolation, or coordinate conversion.

Supports videos whose embedded telemetry ExifTool decodes into these fields;
there is no drone-model restriction in this script. This does not imply every
DJI model or recording mode supplies them. Trimmed/exported videos may have lost
the telemetry track. Missing telemetry or incomplete/invalid GPS samples cause
an error instead of fabricated coordinates. GPS fix quality is not inferred.

No image extraction, orientation, calibration, reconstruction, or training.
