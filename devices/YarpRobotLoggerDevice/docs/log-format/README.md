# Robot log format

This folder specifies the format of the logs (a `.mat` file plus optional videos and image folders) created by the [`YarpRobotLoggerDevice`](../../README.md) and read by the [`robot-log-visualizer`](https://github.com/gbionics/robot-log-visualizer).

The format is versioned independently of `bipedal-locomotion-framework`, so that the same format can also be written by other producers and the consumers do not need to know which software created a log.

| Format version | Specification | `version` variable | Written by |
|:---:|:---:|:---:|:---|
| `v0.1.1` | [v0.1.1.md](./v0.1.1.md) | `"0.1.1"` | `bipedal-locomotion-framework` `v0.31.0` or newer |
| `v0.1.0` | [v0.1.0.md](./v0.1.0.md) | `"0.1.0"` | `bipedal-locomotion-framework` `v0.29.0` to `v0.30.0` |
| `v0.0.1` | [v0.0.1.md](./v0.0.1.md) | missing | `bipedal-locomotion-framework` up to `v0.28.0` |

## How to get the version of a log
Starting from `v0.1.0`, each `.mat` file contains the top-level variable `version`, a char array with the version of the format in the `MAJOR.MINOR.PATCH` form, without the leading `v` (e.g., `0.1.0`). The variable is stored next to the root struct of the log, not inside it. A file without `version` follows the format `v0.0.1`.

```python
import h5py

with h5py.File("robot_logger_device_2026_10_15_12_15_35.mat", "r") as file:
    version = file.get("version")
    # MATLAB char arrays are stored as uint16 (UTF-16) code units
    version = "".join(chr(c) for c in version[()].ravel()) if version is not None else "0.0.1"
```

```matlab
variables = who("-file", "robot_logger_device_2026_10_15_12_15_35.mat");
if ismember("version", variables)
    load("robot_logger_device_2026_10_15_12_15_35.mat", "version");
else
    version = '0.0.1';
end
```

## Versioning rules
- The version changes every time the content of the files, or its meaning, changes in a way that matters to a reader.
- The `MINOR` version is increased if a reader of the previous version is not able to correctly interpret the new files (e.g., a field changes meaning, as `yarp_robot_name` between `v0.0.1` and `v0.1.0`). The `PATCH` version is increased for additions that a reader of the previous version can ignore (e.g., a new channel). The `MAJOR` version will be increased to `1` once the format is considered stable.
- Every new version must be documented in this folder, listing the differences with respect to the previous one, and the version written by the producers must be updated accordingly (for the `YarpRobotLoggerDevice` see `TelemetryBuffer::logFormatVersion`).
