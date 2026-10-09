# YARPRobotLoggerDevice

The **YARPRobotLoggerDevice** is a YARP device based on `YarpSensorBridge` and [`robometry`](https://github.com/robotology/robometry) that allows logging data from robot sensors and actuators in a mat file.

## Configuration Parameters
The logger is currently supported only for the robots listed in the [application folder](./app/robots). Each robot folder contains:

- `launch-yarp-robot-logger.xml`: Configuration parameters for the `yarprobotinterface` to launch the logger device and associated devices.
- `yarp-robot-logger.xml`: Configuration parameters for the logger device.
- `blf-yarp-robot-logger-interfaces`: Folder containing all interfaces used by the logger device to log data.

The robot model associated to the logged data is set with the `robot_model_uri` parameter. Any URI supported by [`resolve-robotics-uri-py`](https://github.com/ami-iit/resolve-robotics-uri-py) can be used, e.g., `package://ergoCub/robots/ergoCubSN001/model.urdf`. The URI is stored in the `yarp_robot_name` field of the mat file and of the real time stream, and it is used by the [`robot-log-visualizer`](https://github.com/ami-iit/robot-log-visualizer) to load the model. The `YARP_ROBOT_NAME` environment variable is not used.

Each mat file also contains the `version` variable storing the version of `bipedal-locomotion-framework` used to create it, as returned by `git describe --tags --dirty` at build time (e.g., `0.27.0-7-ge5762801b-dirty`). The `robot-log-visualizer` loads the robot model only for files whose `version` is `0.28.0` or newer; files created with older versions (or without `version`) are opened without the robot model. Note that development builds after the `v0.27.0` tag report a `0.27.0-*` version until the `v0.28.0` tag is created.

```xml
<param name="robot_model_uri">package://ergoCub/robots/ergoCubSN001/model.urdf</param>
```

### Minimal configuration
Most of the parameters are optional, and what is already described by the attached devices does not need to be repeated:
- the joints are the ones of the attached control board, in the same order (`joints_list` is optional);
- if the group `RobotSensorBridge/InertialSensors` is not provided, all the inertial sensors of the attached multiple analog sensors devices are logged;
- if the group `RobotCameraBridge` is not provided, the attached devices exposing `IRGBDSensor` are logged as rgbd cameras and the ones exposing `IFrameGrabberImage` as rgb cameras. The camera options (`rgb_cameras_fps`, `rgb_cameras_rgb_save_mode`, `rgbd_cameras_fps`, `rgbd_cameras_rgb_save_mode`, `rgbd_cameras_depth_save_mode`, `rgbd_cameras_depth_scale`) accept a single value used for all the cameras, and their default values are `30`, `"video"` and `1000` (depth in millimeters);
- for each exogenous signal only `remote` is required: `signal_name` defaults to the name of the group, `local` to `<port_prefix>/exogenous_signals/<signal_name>` and `carrier` to `udp`;
- the real time streaming is disabled by default (`enable_real_time_logging`) and, if `REAL_TIME_STREAMING` is not provided, it uses the port `<port_prefix>/rt_logging`;
- the `stream_*` flags of `RobotSensorBridge` are `false` by default (except `stream_joint_accelerations`), so only the enabled ones need to be listed.

```xml
<device name="yarp-robot-logger" type="YarpRobotLoggerDevice">
  <param name="robot_model_uri">package://ergoCub/robots/ergoCubSN001/model.urdf</param>

  <group name="ExogenousSignals">
    <param name="vectors_collection_exogenous_inputs">("balancing")</param>
    <group name="balancing">
      <param name="remote">"/balancing-controller/logger"</param>
    </group>
  </group>

  <group name="RobotSensorBridge">
    <param name="stream_joint_states">true</param>
    <param name="stream_motor_states">true</param>
    <param name="stream_inertials">true</param>
  </group>

  <action phase="startup" level="15" type="attach">
    <paramlist name="networks">
      <elem name="all_joints">all_joints_mc</elem>
      <elem name="imu">imu_client</elem>
      <elem name="realsense">realsense</elem>
    </paramlist>
  </action>
  <action phase="shutdown" level="2" type="detach" />
</device>
```

## How to Use the Logger
To use the logger, launch the `yarprobotinterface` with the `launch-yarp-robot-logger.xml` configuration file:

```console
yarprobotinterface --config launch-yarp-robot-logger.xml
```
When you close the yarprobotinterface, the logger will save the logged data in a mat file. Additionally, a md file will contain information about the software version in the robot setup. If video recording is enabled, a mp4 file with the video recording will also be generated. All these files will be saved in the folder specified by the `log_folder` parameter of the `Telemetry` group (the working directory in which `yarprobotinterface` has been launched if not provided).

## Use the logger as always-on telemetry
The device can run in the same `yarprobotinterface` that opens the robot devices, attaching to them directly, and start recording as soon as the robot starts (`auto_start_logging` set to `true`).

```xml
<group name="Telemetry">
  <!-- Folder where the files are saved. It is created if it does not exist. '~' is expanded. -->
  <param name="log_folder">~/telemetry</param>
  <!-- A new file is saved every save_period seconds -->
  <param name="save_period">300.0</param>
</group>
```

- The files are written by a separate thread while the data keeps being logged, so no sample is lost while saving.
- The exogenous signals are monitored every second. When the application streaming a signal is closed the logger detects it, and it reconnects as soon as the port is available again. For each exogenous signal the channel `exogenous_signals_connection_id::<signal_name>` stores, for each logged sample, the index of the connection (1 for the first connection, 2 after the first reconnection, ...). If the structure of a signal changes after a reconnection (e.g., different vector size), the data is stored in a new channel with the suffix `_connection_<index>`.

## Cameras
Each camera stream is acquired and written by two separate threads, so a slow encoding does not affect the acquisition. For each saved image, the channel `camera::<camera>::<rgb|depth>` stores its index in the video (or frames folder) and its time. The index restarts from zero in each video, hence the time of the first image of `<file>_<camera>_rgb.mp4` is the time associated to the index `0` in `<file>.mat`.

The videos are written with [FFmpeg](https://ffmpeg.org/), which is required to compile the device:
- the rgb videos are encoded in H.264 (`libx264`, `libopenh264` or `mpeg4`, the encoder can be chosen with the `video_encoder` parameter) and stored in fragmented mp4 files, readable also if the logger crashes;
- the depth videos are stored with the lossless FFV1 codec (16 bit) in mkv files;
- each image is stored with its own timestamp (variable frame rate), so the video is aligned with the other signals also if some images are dropped.

## How to log exogenous data
The `YarpRobotLoggerDevice` can also log exogenous data, i.e., data not directly provided by the robot sensors and actuators. To do this:
1. modify the `yarp-robot-logger.xml` file to specify the exogenous data to log
2. modify the application that streams the exogenous data

### Modification `yarp-robot-logger.xml` configuration file
Modify the [`ExogenousSignalGroup` in the `yarp-robot-logger.xml` file](https://github.com/ami-iit/bipedal-locomotion-framework/blob/a3a8e9cb8a0c3532db81d814d4851009f8134195/devices/YarpRobotLoggerDevice/app/robots/ergoCubSN000/yarp-robot-logger.xml#L27-L37) to log the data streamed by an application that allows the robot to balance:
   ```xml
   <group name="ExogenousSignals">
     <!-- List containing the names of exogenous signals. Each name should be associated to a sub-group -->
     <param name="vectors_collection_exogenous_inputs">("balancing")</param>
     <param name="vectors_exogenous_inputs">()</param>

      <!-- Sub-group containing the information about the exogenous signal "balancing" -->
     <group name="balancing">
        <!-- Name of the port opened by the logger used to retrieve the exogenous signal data -->
        <param name="local">"/yarp-robot-logger/exogenous_signals/balancing"</param>

        <!-- Name of the port opened by the application used to stream the exogenous signal data -->
        <param name="remote">"/balancing-controller/logger"</param>

        <!-- Name of the exogenous signal (this will be the name of the matlab struct containing all the data associated to the exogenous signal) -->
        <param name="signal_name">"balancing"</param>

        <!-- Carrier used in the port connection -->
        <param name="carrier">"udp"</param>
    </group>
   </group>
   ```

### Stream exogenous data
You need to modify the application that streams the exogenous data to open a port with the name specified in the `remote` parameter of the `balancing` sub-group. For example, if you want to stream the data from the your `balancing` application you need to use `BipedalLocomotion::YarpUtilities::VectorsCollectionServer` class as follows
#### C++
If your application is written in C++ you can use the `BipedalLocomotion::YarpUtilities::VectorsCollectionServer` class as follows

```c++
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

class Module
{
    BipedalLocomotion::YarpUtilities::VectorsCollectionServer m_vectorsCollectionServer; /**< Logger server. */
public:
    // all the other functions you need
}
```
The `m_vectorsCollectionServer` helps you to handle the data you want to send and to populate the metadata. To use this functionality, call `BipedalLocomotion::YarpUtilities::VectorsCollectionServer::populateMetadata` during the configuration phase. Once you have finished populating the metadata you should call `BipedalLocomotion::YarpUtilities::VectorsCollectionServer::finalizeMetadata`
```c++
//This code should go into the configuration phase
auto loggerOption = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>(rf);
if (!m_vectorsCollectionServer.initialize(loggerOption->getGroup("LOGGER")))
{
    log()->error("[BalancingController::configure] Unable to configure the server.");
    return false;
}

m_vectorsCollectionServer.populateMetadata("dcm::position::measured", {"x", "y"});
m_vectorsCollectionServer.populateMetadata("dcm::position::desired", {"x", "y"});

m_vectorsCollectionServer.finalizeMetadata(); // this should be called only once
```
In the main loop, add the following code to prepare and populate the data:

```c++
m_vectorsCollectionServer.prepareData(); // required to prepare the data to be sent
m_vectorsCollectionServer.clearData(); // optional see the documentation

// DCM
m_vectorsCollectionServer.populateData("dcm::position::measured", <signal>);
m_vectorsCollectionServer.populateData("dcm::position::desired", <signal>);

m_vectorsCollectionServer.sendData();
```

**Note:** Replace `<signal>` with the actual data you want to log.


#### Python
If your application is written in Python you can use the `BipedalLocomotion.yarp_utilities.VectorsCollectionServer` class as follows
```python
import bipedal_locomotion_framework as blf

class Module:
    def __init__(self):
        self.vectors_collection_server = blf.yarp_utilities.VectorsCollectionServer() # Logger server.
        # all the other functions you need
```
The `vectors_collection_server` helps you to handle the data you want to send and to populate the metadata. To use this functionality, call `BipedalLocomotion.yarp_utilities.VectorsCollectionServer.populate_metadata` during the configuration phase. Once you have finished populating the metadata you should call `BipedalLocomotion.yarp_utilities.VectorsCollectionServer.finalize_metadata`
```python
#This code should go into the configuration phase
logger_option = blf.parameters_handler.StdParametersHandler()
logger_option.set_parameter_string("remote", "/test/log")
if not self.vectors_collection_server.initialize(logger_option):
    blf.log().error("[BalancingController::configure] Unable to configure the server.")
    raise RuntimeError("Unable to configure the server.")

# populate the metadata
self.vectors_collection_server.populate_metadata("dcm::position::measured", ["x", "y"])
self.vectors_collection_server.populate_metadata("dcm::position::desired", ["x", "y"])

self.vectors_collection_server.finalize_metadata() # this should be called only once when the metadata are ready
```
In the main loop, add the following code to prepare and populate the data:
```python
self.vectors_collection_server.prepare_data() # required to prepare the data to be sent
self.vectors_collection_server.clear_data() # optional see the documentation
self.vectors_collection_server.populate_data("dcm::position::measured", <signal>)
self.vectors_collection_server.populate_data("dcm::position::desired", <signal>)
self.vectors_collection_server.send_data()
```
**Note:** Replace `<signal>` with the actual data you want to log.

## How to visualize the logged data
To visualize the logged data you can use [robot-log-visualizer](https://github.com/ami-iit/robot-log-visualizer). To use the `robot-log-visualizer` you can follow the instructions in the [README](https://github.com/ami-iit/robot-log-visualizer/blob/main/README.md) file.

Once you have installed the `robot-log-visualizer` you can open it from the command line with the following command:
```console
robot-log-visualizer
```
Then, you can open the mat file generated by the logger and explore the logged data as in the following video:

[robot-log-visualizer.webm](https://github.com/ami-iit/robot-log-visualizer/assets/16744101/3fd5c516-da17-4efa-b83b-392b5ce1383b)
