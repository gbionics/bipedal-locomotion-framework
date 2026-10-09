/**
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_CAMERA_BRIDGE_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_CAMERA_BRIDGE_H

// std
#include <memory>

// YARP
#include <yarp/dev/PolyDriverList.h>

#include <BipedalLocomotion/RobotInterface/ICameraBridge.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{


/**
 * YarpCameraBridge Yarp implementation of the ICameraBridge interface
 * Currently available interfaces
 * - Depth Cameras through RGBD sensor interface
 * - Camera images through OpenCV Grabber interface
 *
 * The YarpCameraBridge expects a list of device drivers through the yarp::dev::PolyDriverList object.
 * Each PolyDriver object in the list is compared with the configured sensor names and the assumptions listed below
 * to infer the sensor types and relevant interfaces in order to to read the relevant data.
 *
 * MAJOR ASSUMPTIONS
 * - Every sensor unit(device driver) attached to this Bridge is identified by a unique name
 * - The images are available through a FrameGrabber interface (RGB only) and a RGBD interface (RGB and Depth).
 * - The current internal design (read all sensors in a serial fashion) may not be suitable for a heavy measurement set
 *
 * The parameters for writing the configuration file for this class is given as,
 * |     Group                  |         Parameter               | Type               |                   Description                   |   Mandatory    |
 * |:--------------------------:|:-------------------------------:|:------------------:|:---------------------------------------------- :|:--------------:|
 * |Cameras                     |                                 |                    |Expects cameras to be opened either as remote frame grabber ("RemoteFrameGrabber") with IFrameGrabber interface or rgbd sensor ("RGBDSensorClient") with IRGBDSensor interface | Yes |
 * |                            |rgbd_cameras_list                | vector of strings  |list containing the devices opened as RGBDSensorClients containing the IRGBD sensor interface      | Yes |
 * |                            |rgbd_image_width                 | vector of integers |list containing the color image width dimensions of RGBD cameras. The list must be the same size and order as rgbd_list. If not provided, the size of the image is taken from the YarpInterface after calling `YarpCameraBridge::setDriversList` | No |
 * |                            |rgbd_image_height                | vector of integers |list containing the color image height dimensions of RGBD cameras. The list must be the same size and order as rgbd_list. If not provided the size of the image is taken from the YarpInterface after calling `YarpCameraBridge::setDriversList` | No |
 * |                            |rgbd_depth_image_width           | vector of integers |list containing the depth image width dimensions of RGBD cameras. The list must be the same size and order as rgbd_list. If not provided, the size of the image is taken from the YarpInterface after calling `YarpCameraBridge::setDriversList` | No |
 * |                            |rgbd_depth_image_height          | vector of integers |list containing the depth image height dimensions of RGBD cameras. The list must be the same size and order as rgbd_list. If not provided the size of the image is taken from the YarpInterface after calling `YarpCameraBridge::setDriversList` | No |
 * |                            |rgb_cameras_list                 | vector of strings  |list containing the devices opened as RemoteFrameGrabber devices containing the IFrameGrabber interface | Yes |
 * |                            |rgb_image_width                  | vector of integers |list containing the image width dimensions of RGB cameras. The list must be the same size and order as rgb_list.  If not provided the size of the image is taken from the YarpInterface after calling `YarpCameraBridge::setDriversList` | No |
 * |                            |rgb_image_height                 | vector of integers |list containing the image height dimensions of RGB cameras. The list must be the same size and order as rgb_list.  If not provided the size of the image is taken from the YarpInterface after calling `YarpCameraBridge::setDriversList` | No |
 *
 */
class YarpCameraBridge : public BipedalLocomotion::RobotInterface::ICameraBridge
{
public:
    /**
     * Constructor
     */
    YarpCameraBridge();

    /**
     * Destructor
     */
    ~YarpCameraBridge();

    /**
     * Initialize estimator
     * @param[in] handler Parameters handler
     */
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler) final;

    /**
     * Set the list of device drivers from which the sensor measurements need to be streamed
     * @param deviceDriversList device drivers holding the pointer to sensor interfaces
     * @return True/False in case of success/failure.
     */
    bool setDriversList(const yarp::dev::PolyDriverList& deviceDriversList);

    /**
     * @brief Determines the validity of the object retrieved with getMetadata()
     * @return True if the object is valid, false otherwise.
     */
    bool isValid() const final;

    /**
     * @brief Get the object.
     * @return a const reference of the requested object.
     */
    [[deprecated("Replaced by getMetaData")]]
    const BipedalLocomotion::RobotInterface::CameraBridgeMetaData& get() const;

    /**
     * Get the stored metadata.
     * @return a const reference to the metadata
     */
    const BipedalLocomotion::RobotInterface::CameraBridgeMetaData& getMetaData() const final;

    /**
     * Get rgb cameras
     * @param[out] rgbCamerasList list of rgb cameras attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getRGBCamerasList(std::vector<std::string>& rgbCamerasList) final;

    /**
     * Get RGBD cameras
     * @param[out] rgbdCamerasList list of depth cameras attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getRGBDCamerasList(std::vector<std::string>& rgbdCamerasList) final;

    /**
     * Get color image from the camera
     * @param[in] camName name of the camera
     * @param[out] colorImg image as cv Mat object
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     *
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "colorImg" to this method
     *
     * @return true/false in case of success/failure
     */
    bool getColorImage(const std::string& camName,
                       cv::Mat& colorImg,
                       std::optional<std::reference_wrapper<double>> receiveTimeInSeconds = {}) final;

    /**
     * Get depth image
     * @param[in] camName name of the gyroscope
     * @param[out] depthImg depth image as cv Mat object
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     *
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "depthImg" to this method
     *
     * @return true/false in case of success/failure
     */
    bool getDepthImage(const std::string& camName,
                       cv::Mat& depthImg,
                       std::optional<std::reference_wrapper<double>> receiveTimeInSeconds = {}) final;

private:
    /** Private implementation */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace RobotInterface
} // namespace BipedalLocomotion


#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_CAMERA_BRIDGE_H
