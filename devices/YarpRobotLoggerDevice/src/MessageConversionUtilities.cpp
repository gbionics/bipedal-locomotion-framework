/**
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

 #include <BipedalLocomotion/MessageConversionUtilities.h>

void BipedalLocomotion::extractMetadata(
    const trintrin::msgs::HumanState& message,
    const std::string& prefix,
    BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata& metadata)
{
    std::string p = prefix + treeDelim;
    metadata.vectors[p + "joints_state" + treeDelim + "positions"] = message.jointNames;
    metadata.vectors[p + "joints_state" + treeDelim + "velocities"] = message.jointNames;

    std::string baseName = !message.baseName.empty() ? message.baseName : "root_link";
    metadata.vectors[p + baseName + treeDelim + "position"] = {"x", "y", "z"};

    // Using XYZW convention to be coherent with the robot-log-visualizer
    metadata.vectors[p + baseName + treeDelim + "orientation"] = {"qx", "qy", "qz", "qw"};
    metadata.vectors[p + baseName + treeDelim + "velocity"] = {"vx", "vy", "vz", "wx", "wy", "wz"};

    metadata.vectors[p + "com" + treeDelim + "position"] = {"x", "y", "z"};
    metadata.vectors[p + "com" + treeDelim + "velocity"] = {"x", "y", "z"};
}

void BipedalLocomotion::extractMetadata(
    const trintrin::msgs::WearableTargets& message,
    const std::string& prefix,
    BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata& metadata)
{
    std::string p = prefix + treeDelim;
    for (const auto& [name, target] : message.targets)
    {
        std::string targetPrefix = p + name + treeDelim;
        // The wearable sensor name and link name should not change,
        // so we store them as metadata for the type (that is also constant)
        metadata.vectors[targetPrefix + "type"]
            = {target.wearableSensorName + "_" + target.linkName};
        metadata.vectors[targetPrefix + "position"] = {"x", "y", "z"};
        metadata.vectors[targetPrefix + "orientation"] = {"qx", "qy", "qz", "qw"};
        metadata.vectors[targetPrefix + "linear_velocity"] = {"x", "y", "z"};
        metadata.vectors[targetPrefix + "angular_velocity"] = {"x", "y", "z"};
        metadata.vectors[targetPrefix + "calibration_world_to_measurement_world" + treeDelim
                         + "position"]
            = {"x", "y", "z"};
        metadata.vectors[targetPrefix + "calibration_world_to_measurement_world" + treeDelim
                         + "orientation"]
            = {"qx", "qy", "qz", "qw"};
        metadata.vectors[targetPrefix + "calibration_measurement_to_link" + treeDelim + "position"]
            = {"x", "y", "z"};
        metadata
            .vectors[targetPrefix + "calibration_measurement_to_link" + treeDelim + "orientation"]
            = {"qx", "qy", "qz", "qw"};
        metadata.vectors[targetPrefix + "position_scale_factor"] = {"x", "y", "z"};
    }
}

void BipedalLocomotion::extractMetadata(
    const trintrin::msgs::WearableData& message,
    const std::string& prefix,
    BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata& metadata)
{
    std::string p = prefix + treeDelim;
    for (auto& [name, acc] : message.accelerometers)
    {
        std::string accelerometerPrefix = p + name + treeDelim;
        metadata.vectors[accelerometerPrefix + "status"]
            = {message.producerName + "_" + acc.info.name + "_status"};
        metadata.vectors[accelerometerPrefix + "data"] = {"x", "y", "z"};
    }
    for (auto& [name, emg] : message.emgSensors)
    {
        std::string emgPrefix = p + name + treeDelim;
        metadata.vectors[emgPrefix + "status"]
            = {message.producerName + "_" + emg.info.name + "_status"};
        std::string dataPrefix = emgPrefix + "data" + treeDelim;
        metadata.vectors[dataPrefix + "value"] = {"value"};
        metadata.vectors[dataPrefix + "normalization"] = {"normalization"};
    }
    for (auto& [name, force3] : message.force3DSensors)
    {
        std::string force3Prefix = p + name + treeDelim;
        metadata.vectors[force3Prefix + "status"]
            = {message.producerName + "_" + force3.info.name + "_status"};
        metadata.vectors[force3Prefix + "data"] = {"x", "y", "z"};
    }
    for (auto& [name, force6] : message.forceTorque6DSensors)
    {
        std::string force6Prefix = p + name + treeDelim;
        metadata.vectors[force6Prefix + "status"]
            = {message.producerName + "_" + force6.info.name + "_status"};
        std::string dataPrefix = force6Prefix + "data" + treeDelim;
        metadata.vectors[dataPrefix + "force"] = {"x", "y", "z"};
        metadata.vectors[dataPrefix + "torque"] = {"x", "y", "z"};
    }
    for (auto& [name, fbacc] : message.freeBodyAccelerationSensors)
    {
        std::string fbaccPrefix = p + name + treeDelim;
        metadata.vectors[fbaccPrefix + "status"]
            = {message.producerName + "_" + fbacc.info.name + "_status"};
        metadata.vectors[fbaccPrefix + "data"] = {"x", "y", "z"};
    }
    for (auto& [name, gyro] : message.gyroscopes)
    {
        std::string gyroscopePrefix = p + name + treeDelim;
        metadata.vectors[gyroscopePrefix + "status"]
            = {message.producerName + "_" + gyro.info.name + "_status"};
        metadata.vectors[gyroscopePrefix + "data"] = {"x", "y", "z"};
    }
    for (auto& [name, magnetometer] : message.magnetometers)
    {
        std::string magnetometerPrefix = p + name + treeDelim;
        metadata.vectors[magnetometerPrefix + "status"]
            = {message.producerName + "_" + magnetometer.info.name + "_status"};
        metadata.vectors[magnetometerPrefix + "data"] = {"x", "y", "z"};
    }
    for (auto& [name, orientation] : message.orientationSensors)
    {
        std::string orientationPrefix = p + name + treeDelim;
        metadata.vectors[orientationPrefix + "status"]
            = {message.producerName + "_" + orientation.info.name + "_status"};
        metadata.vectors[orientationPrefix + "data"] = {"qx", "qy", "qz", "qw"};
    }
    for (auto& [name, pose] : message.poseSensors)
    {
        std::string posePrefix = p + name + treeDelim;
        metadata.vectors[posePrefix + "status"]
            = {message.producerName + "_" + pose.info.name + "_status"};
        std::string dataPrefix = posePrefix + "data" + treeDelim;
        metadata.vectors[dataPrefix + "position"] = {"x", "y", "z"};
        metadata.vectors[dataPrefix + "orientation"] = {"qx", "qy", "qz", "qw"};
    }
    for (auto& [name, position] : message.positionSensors)
    {
        std::string positionPrefix = p + name + treeDelim;
        metadata.vectors[positionPrefix + "status"]
            = {message.producerName + "_" + position.info.name + "_status"};
        metadata.vectors[positionPrefix + "data"] = {"x", "y", "z"};
    }
    for (auto& [name, skin] : message.skinSensors)
    {
        std::string skinPrefix = p + name + treeDelim;
        metadata.vectors[skinPrefix + "status"]
            = {message.producerName + "_" + skin.info.name + "_status"};
        metadata.vectors[skinPrefix + "data"] = {};
        for (std::size_t i = 0; i < skin.data.size(); ++i)
        {
            metadata.vectors[skinPrefix + "data"].push_back("value_" + std::to_string(i));
        }
    }
    for (auto& [name, temp] : message.temperatureSensors)
    {
        std::string temperaturePrefix = p + name + treeDelim;
        metadata.vectors[temperaturePrefix + "status"]
            = {message.producerName + "_" + temp.info.name + "_status"};
        metadata.vectors[temperaturePrefix + "data"] = {"value"};
    }
    for (auto& [name, torque] : message.torque3DSensors)
    {
        std::string torque3dPrefix = p + name + treeDelim;
        metadata.vectors[torque3dPrefix + "status"]
            = {message.producerName + "_" + torque.info.name + "_status"};
        metadata.vectors[torque3dPrefix + "data"] = {"x", "y", "z"};
    }
    for (auto& [name, vlink] : message.virtualLinkKinSensors)
    {
        std::string vlinkPrefix = p + name + treeDelim;
        metadata.vectors[vlinkPrefix + "status"]
            = {message.producerName + "_" + vlink.info.name + "_status"};
        std::string dataPrefix = vlinkPrefix + "data" + treeDelim;
        metadata.vectors[dataPrefix + "position"] = {"x", "y", "z"};
        metadata.vectors[dataPrefix + "orientation"] = {"qx", "qy", "qz", "qw"};
        metadata.vectors[dataPrefix + "linear_velocity"] = {"x", "y", "z"};
        metadata.vectors[dataPrefix + "angular_velocity"] = {"x", "y", "z"};
        metadata.vectors[dataPrefix + "linear_acceleration"] = {"x", "y", "z"};
        metadata.vectors[dataPrefix + "angular_acceleration"] = {"x", "y", "z"};
    }
    for (auto& [name, vjoint] : message.virtualJointKinSensors)
    {
        std::string vjointPrefix = p + name + treeDelim;
        metadata.vectors[vjointPrefix + "status"]
            = {message.producerName + "_" + vjoint.info.name + "_status"};
        std::string dataPrefix = vjointPrefix + "data" + treeDelim;
        metadata.vectors[dataPrefix + "position"] = {"value"};
        metadata.vectors[dataPrefix + "velocity"] = {"value"};
        metadata.vectors[dataPrefix + "acceleration"] = {"value"};
    }
    for (auto& [name, vsjoint] : message.virtualSphericalJointKinSensors)
    {
        std::string vsjointPrefix = p + name + treeDelim;
        metadata.vectors[vsjointPrefix + "status"]
            = {message.producerName + "_" + vsjoint.info.name + "_status"};
        std::string dataPrefix = vsjointPrefix + "data" + treeDelim;
        metadata.vectors[dataPrefix + "angle"] = {"roll", "pitch", "yaw"};
        metadata.vectors[dataPrefix + "velocity"] = {"x", "y", "z"};
        metadata.vectors[dataPrefix + "acceleration"] = {"x", "y", "z"};
    }
}

void BipedalLocomotion::convertToVectorsCollection(
    const trintrin::msgs::HumanState& message,
    const std::string& prefix,
    BipedalLocomotion::YarpUtilities::VectorsCollection& collection)
{
    std::string p = prefix + treeDelim;
    collection.vectors[p + "joints_state" + treeDelim + "positions"] = message.positions;
    collection.vectors[p + "joints_state" + treeDelim + "velocities"] = message.velocities;
    std::string baseName = !message.baseName.empty() ? message.baseName : "root_link";
    collection.vectors[p + baseName + treeDelim + "position"] = {message.baseOriginWRTGlobal.x,
                                                                 message.baseOriginWRTGlobal.y,
                                                                 message.baseOriginWRTGlobal.z};
    collection.vectors[p + baseName + treeDelim + "orientation"]
        = {message.baseOrientationWRTGlobal.imaginary.x,
           message.baseOrientationWRTGlobal.imaginary.y,
           message.baseOrientationWRTGlobal.imaginary.z,
           message.baseOrientationWRTGlobal.w};
    collection.vectors[p + baseName + treeDelim + "velocity"] = message.baseVelocityWRTGlobal;
    collection.vectors[p + "com" + treeDelim + "position"] = {message.CoMPositionWRTGlobal.x,
                                                              message.CoMPositionWRTGlobal.y,
                                                              message.CoMPositionWRTGlobal.z};
    collection.vectors[p + "com" + treeDelim + "velocity"] = {message.CoMVelocityWRTGlobal.x,
                                                              message.CoMVelocityWRTGlobal.y,
                                                              message.CoMVelocityWRTGlobal.z};
}

void BipedalLocomotion::convertToVectorsCollection(
    const trintrin::msgs::WearableTargets& message,
    const std::string& prefix,
    BipedalLocomotion::YarpUtilities::VectorsCollection& collection)
{
    std::string p = prefix + treeDelim;
    for (const auto& [name, target] : message.targets)
    {
        std::string targetPrefix = p + name + treeDelim;
        collection.vectors[targetPrefix + "type"] = {static_cast<double>(target.type)};
        collection.vectors[targetPrefix + "position"]
            = {target.position.x, target.position.y, target.position.z};
        collection.vectors[targetPrefix + "orientation"] = {target.orientation.imaginary.x,
                                                            target.orientation.imaginary.y,
                                                            target.orientation.imaginary.z,
                                                            target.orientation.w};
        collection.vectors[targetPrefix + "linear_velocity"]
            = {target.linearVelocity.x, target.linearVelocity.y, target.linearVelocity.z};
        collection.vectors[targetPrefix + "angular_velocity"]
            = {target.angularVelocity.x, target.angularVelocity.y, target.angularVelocity.z};
        collection.vectors[targetPrefix + "calibration_world_to_measurement_world" + treeDelim
                           + "position"]
            = {target.calibrationWorldToMeasurementWorld.position.x,
               target.calibrationWorldToMeasurementWorld.position.y,
               target.calibrationWorldToMeasurementWorld.position.z};
        collection.vectors[targetPrefix + "calibration_world_to_measurement_world" + treeDelim
                           + "orientation"]
            = {target.calibrationWorldToMeasurementWorld.orientation.imaginary.x,
               target.calibrationWorldToMeasurementWorld.orientation.imaginary.y,
               target.calibrationWorldToMeasurementWorld.orientation.imaginary.z,
               target.calibrationWorldToMeasurementWorld.orientation.w};
        collection
            .vectors[targetPrefix + "calibration_measurement_to_link" + treeDelim + "position"]
            = {target.calibrationMeasurementToLink.position.x,
               target.calibrationMeasurementToLink.position.y,
               target.calibrationMeasurementToLink.position.z};
        collection
            .vectors[targetPrefix + "calibration_measurement_to_link" + treeDelim + "orientation"]
            = {target.calibrationMeasurementToLink.orientation.imaginary.x,
               target.calibrationMeasurementToLink.orientation.imaginary.y,
               target.calibrationMeasurementToLink.orientation.imaginary.z,
               target.calibrationMeasurementToLink.orientation.w};
        collection.vectors[targetPrefix + "position_scale_factor"]
            = {target.positionScaleFactor.x,
               target.positionScaleFactor.y,
               target.positionScaleFactor.z};
    }
}

void BipedalLocomotion::convertToVectorsCollection(
    const trintrin::msgs::WearableData& message,
    const std::string& prefix,
    BipedalLocomotion::YarpUtilities::VectorsCollection& collection)
{
    std::string p = prefix + treeDelim;
    for (auto& [name, acc] : message.accelerometers)
    {
        std::string accelerometerPrefix = p + name + treeDelim;
        collection.vectors[accelerometerPrefix + "status"]
            = {static_cast<double>(acc.info.status)};
        collection.vectors[accelerometerPrefix + "data"] = {acc.data.x, acc.data.y, acc.data.z};
    }
    for (auto& [name, emg] : message.emgSensors)
    {
        std::string emgPrefix = p + name + treeDelim;
        collection.vectors[emgPrefix + "status"] = {static_cast<double>(emg.info.status)};
        std::string dataPrefix = emgPrefix + "data" + treeDelim;
        collection.vectors[dataPrefix + "value"] = {emg.data.value};
        collection.vectors[dataPrefix + "normalization"] = {emg.data.normalization};
    }
    for (auto& [name, force3] : message.force3DSensors)
    {
        std::string force3Prefix = p + name + treeDelim;
        collection.vectors[force3Prefix + "status"] = {static_cast<double>(force3.info.status)};
        collection.vectors[force3Prefix + "data"] = {force3.data.x, force3.data.y, force3.data.z};
    }
    for (auto& [name, force6] : message.forceTorque6DSensors)
    {
        std::string force6Prefix = p + name + treeDelim;
        collection.vectors[force6Prefix + "status"] = {static_cast<double>(force6.info.status)};
        std::string dataPrefix = force6Prefix + "data" + treeDelim;
        collection.vectors[dataPrefix + "force"]
            = {force6.data.force.x, force6.data.force.y, force6.data.force.z};
        collection.vectors[dataPrefix + "torque"]
            = {force6.data.torque.x, force6.data.torque.y, force6.data.torque.z};
    }
    for (auto& [name, fbacc] : message.freeBodyAccelerationSensors)
    {
        std::string fbaccPrefix = p + name + treeDelim;
        collection.vectors[fbaccPrefix + "status"] = {static_cast<double>(fbacc.info.status)};
        collection.vectors[fbaccPrefix + "data"] = {fbacc.data.x, fbacc.data.y, fbacc.data.z};
    }
    for (auto& [name, gyro] : message.gyroscopes)
    {
        std::string gyroscopePrefix = p + name + treeDelim;
        collection.vectors[gyroscopePrefix + "status"] = {static_cast<double>(gyro.info.status)};
        collection.vectors[gyroscopePrefix + "data"] = {gyro.data.x, gyro.data.y, gyro.data.z};
    }
    for (auto& [name, magnetometer] : message.magnetometers)
    {
        std::string magnetometerPrefix = p + name + treeDelim;
        collection.vectors[magnetometerPrefix + "status"]
            = {static_cast<double>(magnetometer.info.status)};
        collection.vectors[magnetometerPrefix + "data"]
            = {magnetometer.data.x, magnetometer.data.y, magnetometer.data.z};
    }
    for (auto& [name, orientation] : message.orientationSensors)
    {
        std::string orientationPrefix = p + name + treeDelim;
        collection.vectors[orientationPrefix + "status"]
            = {static_cast<double>(orientation.info.status)};
        collection.vectors[orientationPrefix + "data"] = {orientation.data.imaginary.x,
                                                          orientation.data.imaginary.y,
                                                          orientation.data.imaginary.z,
                                                          orientation.data.w};
    }
    for (auto& [name, pose] : message.poseSensors)
    {
        std::string posePrefix = p + name + treeDelim;
        collection.vectors[posePrefix + "status"] = {static_cast<double>(pose.info.status)};
        std::string dataPrefix = posePrefix + "data" + treeDelim;
        collection.vectors[dataPrefix + "position"]
            = {pose.data.position.x, pose.data.position.y, pose.data.position.z};
        collection.vectors[dataPrefix + "orientation"] = {pose.data.orientation.imaginary.x,
                                                          pose.data.orientation.imaginary.y,
                                                          pose.data.orientation.imaginary.z,
                                                          pose.data.orientation.w};
    }
    for (auto& [name, position] : message.positionSensors)
    {
        std::string positionPrefix = p + name + treeDelim;
        collection.vectors[positionPrefix + "status"]
            = {static_cast<double>(position.info.status)};
        collection.vectors[positionPrefix + "data"]
            = {position.data.x, position.data.y, position.data.z};
    }
    for (auto& [name, skin] : message.skinSensors)
    {
        std::string skinPrefix = p + name + treeDelim;
        collection.vectors[skinPrefix + "status"] = {static_cast<double>(skin.info.status)};
        collection.vectors[skinPrefix + "data"] = skin.data;
    }
    for (auto& [name, temp] : message.temperatureSensors)
    {
        std::string temperaturePrefix = p + name + treeDelim;
        collection.vectors[temperaturePrefix + "status"] = {static_cast<double>(temp.info.status)};
        collection.vectors[temperaturePrefix + "data"] = {temp.data};
    }
    for (auto& [name, torque] : message.torque3DSensors)
    {
        std::string torque3dPrefix = p + name + treeDelim;
        collection.vectors[torque3dPrefix + "status"] = {static_cast<double>(torque.info.status)};
        collection.vectors[torque3dPrefix + "data"]
            = {torque.data.x, torque.data.y, torque.data.z};
    }
    for (auto& [name, vlink] : message.virtualLinkKinSensors)
    {
        std::string vlinkPrefix = p + name + treeDelim;
        collection.vectors[vlinkPrefix + "status"] = {static_cast<double>(vlink.info.status)};
        std::string dataPrefix = vlinkPrefix + "data" + treeDelim;
        collection.vectors[dataPrefix + "position"]
            = {vlink.data.position.x, vlink.data.position.y, vlink.data.position.z};
        collection.vectors[dataPrefix + "orientation"] = {vlink.data.orientation.imaginary.x,
                                                          vlink.data.orientation.imaginary.y,
                                                          vlink.data.orientation.imaginary.z,
                                                          vlink.data.orientation.w};
        collection.vectors[dataPrefix + "linear_velocity"] = {vlink.data.linearVelocity.x,
                                                              vlink.data.linearVelocity.y,
                                                              vlink.data.linearVelocity.z};
        collection.vectors[dataPrefix + "angular_velocity"] = {vlink.data.angularVelocity.x,
                                                               vlink.data.angularVelocity.y,
                                                               vlink.data.angularVelocity.z};
        collection.vectors[dataPrefix + "linear_acceleration"] = {vlink.data.linearAcceleration.x,
                                                                  vlink.data.linearAcceleration.y,
                                                                  vlink.data.linearAcceleration.z};
        collection.vectors[dataPrefix + "angular_acceleration"]
            = {vlink.data.angularAcceleration.x,
               vlink.data.angularAcceleration.y,
               vlink.data.angularAcceleration.z};
    }
    for (auto& [name, vjoint] : message.virtualJointKinSensors)
    {
        std::string vjointPrefix = p + name + treeDelim;
        collection.vectors[vjointPrefix + "status"] = {static_cast<double>(vjoint.info.status)};
        std::string dataPrefix = vjointPrefix + "data" + treeDelim;
        collection.vectors[dataPrefix + "position"] = {vjoint.data.position};
        collection.vectors[dataPrefix + "velocity"] = {vjoint.data.velocity};
        collection.vectors[dataPrefix + "acceleration"] = {vjoint.data.acceleration};
    }
    for (auto& [name, vsjoint] : message.virtualSphericalJointKinSensors)
    {
        std::string vsjointPrefix = p + name + treeDelim;
        collection.vectors[vsjointPrefix + "status"] = {static_cast<double>(vsjoint.info.status)};
        std::string dataPrefix = vsjointPrefix + "data" + treeDelim;
        collection.vectors[dataPrefix + "angle"]
            = {vsjoint.data.angle.r, vsjoint.data.angle.p, vsjoint.data.angle.y};
        collection.vectors[dataPrefix + "velocity"]
            = {vsjoint.data.velocity.x, vsjoint.data.velocity.y, vsjoint.data.velocity.z};
        collection.vectors[dataPrefix + "acceleration"] = {vsjoint.data.acceleration.x,
                                                           vsjoint.data.acceleration.y,
                                                           vsjoint.data.acceleration.z};
    }
}
