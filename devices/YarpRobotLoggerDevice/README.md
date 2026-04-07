# YARPRobotLoggerDevice

The **YARPRobotLoggerDevice** is a YARP device based on `YarpSensorBridge` and [`robometry`](https://github.com/robotology/robometry) that logs data from robot sensors and actuators into `.mat` files. It also supports logging cameras (as `.mp4` videos or image frames), exogenous signals streamed over YARP ports, text logs, and frame transforms.

## Quick start

Launch the logger with `yarprobotinterface`:

```console
yarprobotinterface --config launch-yarp-robot-logger.xml
```

By default (`auto_start_logging: true`) the logger starts recording immediately. When you close `yarprobotinterface` (e.g. with `Ctrl+C`), the recorded data is **automatically saved** before the device shuts down. All output files (`.mat`, `.mp4`, `.md`) are written to the working directory.

## RPC commands

The logger exposes an RPC port at `<port_prefix>/commands/rpc:i` (default: `/yarp-robot-logger/commands/rpc:i`). You can control recording with the following commands using `yarp rpc`:

```console
yarp rpc /yarp-robot-logger/commands/rpc:i
```

| Command | Description |
|---|---|
| `record` | Start recording. Initializes sensor bridge, cameras, telemetry buffers and begins the periodic acquisition loop. |
| `saveData` | Save the recorded data to a `.mat` file. Recording **continues** after the save completes. Accepts an optional `tag` string that is appended to the filename (e.g. `saveData "walking_test"`). Can be called multiple times during a single recording session. |
| `saveData "<tag>"` | Same as above, with the tag included in the output filename: `robot_logger_device_<tag>_<timestamp>.mat`. |
| `stopRecording` | Stop recording **without saving**. All buffered data is discarded and the device returns to idle mode. |

After `stopRecording`, you can call `record` again to start a new recording session.

### Example session

```console
$ yarp rpc /yarp-robot-logger/commands/rpc:i
>> record
Response: [ok]
# ... robot performs some task ...
>> saveData "walking_trial_1"
Response: [ok]
# data saved to robot_logger_device_walking_trial_1_<timestamp>.mat
# recording continues
>> saveData "walking_trial_2"
Response: [ok]
# data saved to robot_logger_device_walking_trial_2_<timestamp>.mat
# recording still continues
>> stopRecording
Response: [ok]
# recording stopped, no additional data saved
```

### Auto-start mode

When `auto_start_logging` is set to `true` (the default), `record` is called automatically at device startup. If set to `false`, the device starts in idle mode and waits for an explicit `record` RPC command.

## Configuration

The logger is currently supported for the robots listed in the [application folder](./app/robots). Each robot folder contains:

- `launch-yarp-robot-logger.xml`: `yarprobotinterface` configuration that launches the logger and associated devices.
- `yarp-robot-logger.xml`: Logger device parameters.
- `blf-yarp-robot-logger-interfaces/`: Interface configuration files for the sensor bridge.

### Key parameters

| Parameter | Default | Description |
|---|---|---|
| `sampling_period_in_s` | `0.01` | Logging period in seconds. |
| `port_prefix` | `/yarp-robot-logger` | Prefix for all YARP ports opened by the logger. |
| `auto_start_logging` | `true` | Start recording automatically on device startup. |
| `log_robot_data` | `true` | Log joint states, motor states, FT sensors, IMUs, etc. |
| `log_cameras` | `true` | Log camera images as video or frames. |
| `log_text` | `true` | Log YARP text logging messages. |
| `maximum_admissible_time_step` | - | Max time step between samples (skip logging on clock resets). |

## Logging exogenous data

The logger can also record exogenous data, i.e., signals not directly from robot sensors. To do this:

1. Configure the `ExogenousSignals` group in `yarp-robot-logger.xml`.
2. Stream data from your application using `VectorsCollectionServer`.

### Configuration

Add an `ExogenousSignals` group to `yarp-robot-logger.xml`:

```xml
<group name="ExogenousSignals">
  <param name="vectors_collection_exogenous_inputs">("balancing")</param>
  <param name="vectors_exogenous_inputs">()</param>

  <group name="balancing">
    <param name="local">"/yarp-robot-logger/exogenous_signals/balancing"</param>
    <param name="remote">"/balancing-controller/logger"</param>
    <param name="signal_name">"balancing"</param>
    <param name="carrier">"udp"</param>
  </group>
</group>
```

### Streaming from C++

```cpp
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

// Configuration phase
BipedalLocomotion::YarpUtilities::VectorsCollectionServer server;
server.initialize(loggerOption->getGroup("LOGGER"));
server.populateMetadata("dcm::position::measured", {"x", "y"});
server.populateMetadata("dcm::position::desired", {"x", "y"});
server.finalizeMetadata();

// Main loop
server.prepareData();
server.clearData();
server.populateData("dcm::position::measured", measuredSignal);
server.populateData("dcm::position::desired", desiredSignal);
server.sendData();
```

### Streaming from Python

```python
import bipedal_locomotion_framework as blf

# Configuration phase
server = blf.yarp_utilities.VectorsCollectionServer()
handler = blf.parameters_handler.StdParametersHandler()
handler.set_parameter_string("remote", "/balancing-controller/logger")
server.initialize(handler)
server.populate_metadata("dcm::position::measured", ["x", "y"])
server.populate_metadata("dcm::position::desired", ["x", "y"])
server.finalize_metadata()

# Main loop
server.prepare_data()
server.clear_data()
server.populate_data("dcm::position::measured", measured_signal)
server.populate_data("dcm::position::desired", desired_signal)
server.send_data()
```

## Visualizing logged data

Use [robot-log-visualizer](https://github.com/ami-iit/robot-log-visualizer) to explore the `.mat` files:

```console
robot-log-visualizer
```

Then open the `.mat` file generated by the logger. See the [robot-log-visualizer README](https://github.com/ami-iit/robot-log-visualizer/blob/main/README.md) for details.
