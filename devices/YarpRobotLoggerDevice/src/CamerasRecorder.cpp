/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotLogger/CamerasRecorder.h>

using namespace BipedalLocomotion::RobotLogger;
using BipedalLocomotion::RobotInterface::YarpCameraBridge;

namespace
{

class CameraColorSource final : public IImageSource
{
public:
    CameraColorSource(YarpCameraBridge& bridge,
                      std::string camera,
                      std::shared_ptr<std::mutex> mutex)
        : m_bridge(bridge)
        , m_camera(std::move(camera))
        , m_mutex(std::move(mutex))
    {
    }

    bool read(cv::Mat& image) final
    {
        std::lock_guard lock(*m_mutex);
        if (!m_bridge.getColorImage(m_camera, m_buffer))
        {
            return false;
        }
        // the buffer may share the memory with the bridge
        image = m_buffer.clone();
        return true;
    }

private:
    YarpCameraBridge& m_bridge;
    std::string m_camera;
    std::shared_ptr<std::mutex> m_mutex;
    cv::Mat m_buffer;
};

class CameraDepthSource final : public IImageSource
{
public:
    CameraDepthSource(YarpCameraBridge& bridge,
                      std::string camera,
                      std::shared_ptr<std::mutex> mutex,
                      double scale)
        : m_bridge(bridge)
        , m_camera(std::move(camera))
        , m_mutex(std::move(mutex))
        , m_scale(scale)
    {
    }

    bool read(cv::Mat& image) final
    {
        std::lock_guard lock(*m_mutex);
        if (!m_bridge.getDepthImage(m_camera, m_buffer))
        {
            return false;
        }
        m_buffer.convertTo(image, CV_16UC1, m_scale);
        return true;
    }

private:
    YarpCameraBridge& m_bridge;
    std::string m_camera;
    std::shared_ptr<std::mutex> m_mutex;
    double m_scale;
    cv::Mat m_buffer;
};

} // namespace

bool CamerasRecorder::readCamerasOptions(
    std::shared_ptr<const ParametersHandler::IParametersHandler> params,
    const std::vector<std::string>& cameras,
    bool isRGBD)
{
    constexpr auto logPrefix = "[CamerasRecorder::readCamerasOptions]";

    const std::string prefix = isRGBD ? "rgbd_cameras_" : "rgb_cameras_";

    auto parseMode = [logPrefix](const std::string& mode, ImageSaveMode& saveMode) {
        if (mode == "frame")
        {
            saveMode = ImageSaveMode::Frames;
            return true;
        }
        if (mode == "video")
        {
            saveMode = ImageSaveMode::Video;
            return true;
        }
        log()->error("{} The save mode must be either 'frame' or 'video'. Provided: {}.",
                     logPrefix,
                     mode);
        return false;
    };

    std::vector<int> fps;
    std::vector<std::string> rgbSaveModes;
    if (!params->getParameter(prefix + "fps", fps)
        || !params->getParameter(prefix + "rgb_save_mode", rgbSaveModes))
    {
        log()->error("{} Unable to get the parameters '{}fps' and '{}rgb_save_mode'.",
                     logPrefix,
                     prefix,
                     prefix);
        return false;
    }

    std::vector<int> depthScales;
    std::vector<std::string> depthSaveModes;
    if (isRGBD)
    {
        if (!params->getParameter(prefix + "depth_scale", depthScales)
            || !params->getParameter(prefix + "depth_save_mode", depthSaveModes))
        {
            log()->error("{} Unable to get the parameters '{}depth_scale' and "
                         "'{}depth_save_mode'.",
                         logPrefix,
                         prefix,
                         prefix);
            return false;
        }
    }

    const std::size_t size = cameras.size();
    if (fps.size() != size || rgbSaveModes.size() != size
        || (isRGBD && (depthScales.size() != size || depthSaveModes.size() != size)))
    {
        log()->error("{} The size of the '{}*' parameters must be equal to the number of cameras "
                     "({}).",
                     logPrefix,
                     prefix,
                     size);
        return false;
    }

    for (std::size_t i = 0; i < size; i++)
    {
        StreamOptions options;
        if (fps[i] <= 0)
        {
            log()->error("{} The fps of the camera {} must be positive.", logPrefix, cameras[i]);
            return false;
        }
        options.fps = fps[i];

        if (!parseMode(rgbSaveModes[i], options.rgbSaveMode))
        {
            return false;
        }

        if (isRGBD)
        {
            options.hasDepth = true;
            options.depthScale = depthScales[i];
            if (!parseMode(depthSaveModes[i], options.depthSaveMode))
            {
                return false;
            }
        }

        m_cameras[cameras[i]] = options;
        m_cameraMutexes[cameras[i]] = std::make_shared<std::mutex>();
    }

    return true;
}

bool CamerasRecorder::initialize(std::shared_ptr<const ParametersHandler::IParametersHandler> params)
{
    constexpr auto logPrefix = "[CamerasRecorder::initialize]";

    auto group = params->getGroup("RobotCameraBridge").lock();
    if (group == nullptr)
    {
        log()->error("{} The 'RobotCameraBridge' group is not provided.", logPrefix);
        return false;
    }

    m_bridge = std::make_unique<YarpCameraBridge>();
    if (!m_bridge->initialize(group))
    {
        log()->error("{} Unable to configure the camera bridge.", logPrefix);
        return false;
    }

    const auto& metadata = m_bridge->getMetaData();
    if (metadata.bridgeOptions.isRGBCameraEnabled
        && !this->readCamerasOptions(params, metadata.sensorsList.rgbCamerasList, false))
    {
        return false;
    }

    if (metadata.bridgeOptions.isRGBDCameraEnabled
        && !this->readCamerasOptions(params, metadata.sensorsList.rgbdCamerasList, true))
    {
        return false;
    }

    if (params->getParameter("video_codec_code", m_encoderOptions.fourcc)
        && m_encoderOptions.fourcc.size() != 4)
    {
        log()->error("{} The parameter 'video_codec_code' must contain 4 characters.", logPrefix);
        return false;
    }

    params->getParameter("video_encoder", m_encoderOptions.encoder);

    return true;
}

bool CamerasRecorder::setDriversList(const yarp::dev::PolyDriverList& poly)
{
    if (!m_bridge->setDriversList(poly))
    {
        log()->error("[CamerasRecorder::setDriversList] Could not attach the drivers list to the "
                     "camera bridge.");
        return false;
    }
    return true;
}

bool CamerasRecorder::start(DataStorage& storage)
{
    m_recorders.clear();

    for (const auto& [camera, stream] : m_cameras)
    {
        const auto& mutex = m_cameraMutexes.at(camera);

        ImageRecorderOptions options;
        options.name = camera;
        options.fps = stream.fps;
        options.encoder = m_encoderOptions;
        options.encoder.fps = stream.fps;

        options.imageType = "rgb";
        options.channel = "camera::" + camera + "::rgb";
        options.saveMode = stream.rgbSaveMode;
        options.isDepth = false;
        m_recorders.push_back(
            std::make_unique<ImageRecorder>(options,
                                            std::make_unique<CameraColorSource>(*m_bridge,
                                                                                camera,
                                                                                mutex),
                                            storage));

        if (stream.hasDepth)
        {
            options.imageType = "depth";
            options.channel = "camera::" + camera + "::depth";
            options.saveMode = stream.depthSaveMode;
            options.isDepth = true;
            m_recorders.push_back(std::make_unique<ImageRecorder>(
                options,
                std::make_unique<CameraDepthSource>(*m_bridge, camera, mutex, stream.depthScale),
                storage));
        }
    }

    for (auto& recorder : m_recorders)
    {
        if (!recorder->start())
        {
            return false;
        }
    }
    return true;
}

std::vector<ImageRecorder*> CamerasRecorder::recorders()
{
    std::vector<ImageRecorder*> recorders;
    for (auto& recorder : m_recorders)
    {
        recorders.push_back(recorder.get());
    }
    return recorders;
}

void CamerasRecorder::stop()
{
    m_recorders.clear();
}
