/**
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */


#ifndef BIPEDAL_LOCOMOTION_PERCEPTION_CAPTURE_REALSENSE_H
#define BIPEDAL_LOCOMOTION_PERCEPTION_CAPTURE_REALSENSE_H

#include <BipedalLocomotion/RobotInterface/ICameraBridge.h>
#include <BipedalLocomotion/RobotInterface/IPointCloudBridge.h>

namespace BipedalLocomotion {

namespace Perception {

namespace Capture {

/**
 * Realsense driver class
 *
 * The following parameters are required to initialize the class
 *
 * |      Parameter Name     |       Type        |             Description                       | Mandatory | Default Value |
 * |:-----------------------:|:-----------------:|:---------------------------------------------:|:---------:|:-------------:|
 * |      `camera_name`      |      `string`     |         name of the camera device             |     No    |   RealSense   |
 * |      `frame_width`      |     `size_t`      |            image frame width                  |     No    |     640       |
*  |     `frame_height`      |     `size_t`      |            image frame height                 |     No    |     480       |
*  |          `fps`          |     `boolean`     |            frames per second                  |     No    |      30       |
*  |      `stream_color`     |     `boolean`     |      flag to enable streaming BGR images      |     No    |     false     |
*  |      `stream_depth`     |     `boolean`     |     flag to enable streaming depth images     |     No    |     false     |
*  |       `stream_ir`       |     `boolean`     |      flag to enable streaming IR images       |     No    |     false     |
*  |       `stream_pcl`      |     `boolean`     |      flag to enable streaming pointcloud      |     No    |     false     |
*  | `align_frames_to_color` |     `boolean`     |    flag to align other images to BGR images   |     No    |     false     |
*/

class RealSense : public BipedalLocomotion::RobotInterface::ICameraBridge,
                  public BipedalLocomotion::RobotInterface::IPointCloudBridge
{
public:
    RealSense();

    ~RealSense();

    bool initialize(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler) final;

   /**
     * @brief Determines the validity of the object retrieved with getMetadata()
     * @return True if the object is valid, false otherwise.
     */
    bool isValid() const final;

    bool getRGBCamerasList(std::vector<std::string>& rgbCamerasList) final;

    bool getRGBDCamerasList(std::vector<std::string>& rgbdCamerasList) final;

    bool getPCLDevicesList(std::vector<std::string>& pclDevList) final;

    // Expose the deprecated overloads hidden by the declarations below.
    using ICameraBridge::getColorImage;
    using ICameraBridge::getDepthImage;
    using IPointCloudBridge::getPointCloud;

    bool getColorImage(
        const std::string& camName,
        cv::Mat& colorImg,
        std::optional<std::reference_wrapper<std::chrono::nanoseconds>> receiveTime = {}) final;

    bool getDepthImage(
        const std::string& camName,
        cv::Mat& depthImg,
        std::optional<std::reference_wrapper<std::chrono::nanoseconds>> receiveTime = {}) final;

    bool getColorizedDepthImage(
        const std::string& camName,
        cv::Mat& depthImg,
        std::optional<std::reference_wrapper<std::chrono::nanoseconds>> receiveTime = {});

    [[deprecated("Use the getColorizedDepthImage overload taking the receive time as "
                 "std::chrono::nanoseconds.")]]
    bool getColorizedDepthImage(const std::string& camName,
                                cv::Mat& depthImg,
                                std::optional<std::reference_wrapper<double>> receiveTimeInSeconds);

    bool getInfraredImage(
        const std::string& camName,
        cv::Mat& irImage,
        std::optional<std::reference_wrapper<std::chrono::nanoseconds>> receiveTime = {});

    [[deprecated("Use the getInfraredImage overload taking the receive time as "
                 "std::chrono::nanoseconds.")]]
    bool getInfraredImage(const std::string& camName,
                          cv::Mat& irImage,
                          std::optional<std::reference_wrapper<double>> receiveTimeInSeconds);

    bool getPointCloud(
        const std::string& pclDevName,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPointCloud,
        std::optional<std::reference_wrapper<std::chrono::nanoseconds>> receiveTime = {}) final;

    /**
     * Get the stored metadata.
     * @return a const reference to the metadata
     */
    const BipedalLocomotion::RobotInterface::CameraBridgeMetaData& getMetaData() const final;

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace Capture
} // namespace Perception
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PERCEPTION_CAPTURE_REALSENSE_H
