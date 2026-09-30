/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_CAMERAS_RECORDER_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_CAMERAS_RECORDER_H

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <yarp/dev/PolyDriverList.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/RobotInterface/YarpCameraBridge.h>

#include <BipedalLocomotion/RobotLogger/DataStorage.h>
#include <BipedalLocomotion/RobotLogger/ImageRecorder.h>

namespace BipedalLocomotion
{
namespace RobotLogger
{

/**
 * CamerasRecorder records the rgb and rgbd cameras attached to the device through a
 * YarpCameraBridge. Each image stream is recorded by an ImageRecorder.
 */
class CamerasRecorder
{
public:
    /**
     * @param params device parameters. They contain the group `RobotCameraBridge` and the
     * parameters `rgb_cameras_fps`, `rgb_cameras_rgb_save_mode`, `rgbd_cameras_fps`,
     * `rgbd_cameras_depth_scale`, `rgbd_cameras_rgb_save_mode`, `rgbd_cameras_depth_save_mode`
     * and the optional `video_codec_code` (fourcc used by OpenCV) and `video_encoder` (FFmpeg
     * encoder).
     */
    bool initialize(std::shared_ptr<const ParametersHandler::IParametersHandler> params);

    bool setDriversList(const yarp::dev::PolyDriverList& poly);

    /** Start the recording. It must be called at the beginning of each recording session. */
    bool start(DataStorage& storage);

    std::vector<ImageRecorder*> recorders();

    /** Stop and destroy the recorders. */
    void stop();

private:
    struct StreamOptions
    {
        double fps{0};
        ImageSaveMode rgbSaveMode{ImageSaveMode::Video};
        bool hasDepth{false};
        ImageSaveMode depthSaveMode{ImageSaveMode::Frames};
        double depthScale{1.0};
    };

    bool readCamerasOptions(std::shared_ptr<const ParametersHandler::IParametersHandler> params,
                            const std::vector<std::string>& cameras,
                            bool isRGBD);

    std::unique_ptr<RobotInterface::YarpCameraBridge> m_bridge;
    std::unordered_map<std::string, StreamOptions> m_cameras;
    std::unordered_map<std::string, std::shared_ptr<std::mutex>> m_cameraMutexes;
    VideoEncoderOptions m_encoderOptions;
    std::vector<std::unique_ptr<ImageRecorder>> m_recorders;
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_CAMERAS_RECORDER_H
