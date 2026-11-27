/**
 * @file DinrailCameraBridge.h
 * @copyright Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_DINRAIL_CAMERA_BRIDGE_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_DINRAIL_CAMERA_BRIDGE_H

#include <memory>
#include <vector>

#include <BipedalLocomotion/RobotInterface/DinrailHelper.h>
#include <BipedalLocomotion/RobotInterface/ICameraBridge.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 * DinrailCameraBridge is the dinrail implementation of the ICameraBridge interface, the
 * counterpart of YarpCameraBridge. It reads the RGBD cameras through the dinrail::IRGBDSensor
 * interface, implemented for example by the dinrail RGBD sensor client (see
 * constructDinrailRGBDSensorClient()) or directly by a camera device.
 *
 * The getters of dinrail::IRGBDSensor never block: they return the latest images received, hence
 * reading the images of the bridge never waits for the camera.
 *
 * The parameters for writing the configuration file for this class are given as,
 * |  Group  |         Parameter         | Type               |                   Description                   | Mandatory |
 * |:-------:|:-------------------------:|:------------------:|:-----------------------------------------------:|:---------:|
 * |         | `stream_cameras`          | boolean            | If false or missing, no camera is configured.   | No        |
 * | Cameras | `rgbd_cameras_list`       | vector of strings  | List of the keys of the devices, passed to setDevicesList(), implementing dinrail::IRGBDSensor. | Yes |
 * | Cameras | `rgbd_image_width`        | vector of integers | Color image width of the RGBD cameras, same size and order as `rgbd_cameras_list`. If not provided, it is retrieved from the intrinsic parameters of the camera. | No |
 * | Cameras | `rgbd_image_height`       | vector of integers | Color image height of the RGBD cameras. | No |
 * | Cameras | `rgbd_depth_image_width`  | vector of integers | Depth image width of the RGBD cameras. If not provided, it is retrieved from the intrinsic parameters of the camera. | No |
 * | Cameras | `rgbd_depth_image_height` | vector of integers | Depth image height of the RGBD cameras. | No |
 *
 * dinrail does not provide an interface for RGB-only cameras, hence `rgb_cameras_list` is not
 * supported. The image dimensions are only used to preallocate the image buffers.
 */
class DinrailCameraBridge : public ICameraBridge
{
public:
    DinrailCameraBridge();
    ~DinrailCameraBridge() override;

    /**
     * Initialize the bridge.
     * @param[in] handler Parameters handler.
     * @return True/False in case of success/failure.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) final;

    /**
     * Set the list of devices from which the images are read. The bridge shares the ownership of
     * the devices, which are kept open at least as long as the bridge is.
     * @param devices list of devices, the configured cameras are identified by their key.
     * @return True/False in case of success/failure.
     */
    bool setDevicesList(const std::vector<DinrailDeviceDescriptor>& devices);

    bool isValid() const final;

    const CameraBridgeMetaData& getMetaData() const final;

    bool getRGBCamerasList(std::vector<std::string>& rgbCamerasList) final;

    bool getRGBDCamerasList(std::vector<std::string>& rgbdCamerasList) final;

    // Expose the deprecated overloads hidden by the declarations below.
    using ICameraBridge::getColorImage;
    using ICameraBridge::getDepthImage;

    /**
     * Get the latest color image of the camera, as a BGR image.
     * @param[in] camName name of the camera.
     * @param[out] colorImg image as cv Mat object.
     * @param[out] receiveTime time at which the image was read for the first time by the bridge,
     * according to BipedalLocomotion::clock(). Reading again the same frame returns its original
     * receive time, hence a new frame is detected by a change of receiveTime.
     *
     * @note colorImg may share memory with an internal buffer. The data remain valid until the
     * next read from the same camera. Clone colorImg if it must be stored.
     *
     * @return True/False in case of success/failure. Before the first image is received by the
     * camera, false is returned without logging an error.
     */
    bool getColorImage(const std::string& camName,
                       cv::Mat& colorImg,
                       std::optional<std::reference_wrapper<std::chrono::nanoseconds>> receiveTime
                       = {}) final;

    /**
     * Get the latest depth image of the camera, in meters (CV_32FC1), zero for the invalid pixels.
     * @param[in] camName name of the camera.
     * @param[out] depthImg depth image as cv Mat object.
     * @param[out] receiveTime time at which the image was read for the first time by the bridge,
     * according to BipedalLocomotion::clock(). Reading again the same frame returns its original
     * receive time, hence a new frame is detected by a change of receiveTime.
     *
     * @note depthImg shares memory with an internal buffer. The data remain valid until the next
     * depth read from the same camera. Clone depthImg if it must be stored.
     *
     * @return True/False in case of success/failure. Before the first image is received by the
     * camera, false is returned without logging an error.
     */
    bool getDepthImage(const std::string& camName,
                       cv::Mat& depthImg,
                       std::optional<std::reference_wrapper<std::chrono::nanoseconds>> receiveTime
                       = {}) final;

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_DINRAIL_CAMERA_BRIDGE_H
