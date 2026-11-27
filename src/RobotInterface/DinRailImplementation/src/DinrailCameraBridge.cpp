/**
 * @file DinrailCameraBridge.cpp
 * @copyright Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotInterface/DinrailCameraBridge.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <dinrail/CameraTypes.h>
#include <dinrail/IRGBDSensor.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <optional>
#include <string_view>
#include <unordered_map>

using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::ParametersHandler;

namespace
{

using Dimensions = std::pair<std::size_t, std::size_t>;

/**
 * Update the receive time of a frame. The time is updated only for a new frame: the dinrail
 * getters return the latest frame received until a new one arrives.
 */
void updateReceiveTime(const dinrail::Stamp& newStamp,
                       std::optional<dinrail::Stamp>& stamp,
                       std::chrono::nanoseconds& time)
{
    if (!stamp || stamp->sequenceNumber != newStamp.sequenceNumber || stamp->time != newStamp.time)
    {
        time = BipedalLocomotion::clock().now();
    }
    stamp = newStamp;
}

void setReceiveTime(std::optional<std::reference_wrapper<std::chrono::nanoseconds>>& receiveTime,
                    const std::chrono::nanoseconds& time)
{
    if (receiveTime)
    {
        receiveTime->get() = time;
    }
}

} // namespace

struct DinrailCameraBridge::Impl
{
    struct Camera
    {
        /** Shared with the user, keeps the device open. */
        std::shared_ptr<dinrail::Device> device;
        /** Interface of the device, valid as long as the device is open. */
        dinrail::IRGBDSensor* sensor{nullptr};

        dinrail::Image rgb;
        cv::Mat bgr;
        std::optional<dinrail::Stamp> rgbStamp;
        std::chrono::nanoseconds rgbTime{0};

        dinrail::DepthImage depth;
        std::optional<dinrail::Stamp> depthStamp;
        std::chrono::nanoseconds depthTime{0};
    };

    CameraBridgeMetaData metaData;
    std::unordered_map<std::string, Camera> cameras;
    bool bridgeInitialized{false};
    bool devicesAttached{false};

    bool checkValid(std::string_view logPrefix) const
    {
        if (!(bridgeInitialized && devicesAttached))
        {
            log()->error("{} CameraBridge is not ready. Please initialize and set the devices "
                         "list.",
                         logPrefix);
            return false;
        }
        return true;
    }

    Camera* findCamera(std::string_view logPrefix, const std::string& name)
    {
        auto camera = cameras.find(name);
        if (camera == cameras.end())
        {
            log()->error("{} Unable to find the camera named {}.", logPrefix, name);
            return nullptr;
        }
        return &camera->second;
    }

    /**
     * Check the status returned by a getter. Unavailable means that no image has been received
     * yet, which is expected while the camera starts, hence it is not reported as an error.
     */
    static bool checkReadStatus(std::string_view logPrefix,
                                const std::string& camName,
                                const dinrail::Status& status)
    {
        if (status.ok())
        {
            return true;
        }
        if (status.code() != dinrail::StatusCode::Unavailable)
        {
            log()->error("{} Unable to read the image from the camera named {}: {}.",
                         logPrefix,
                         camName,
                         status.ToStringView());
        }
        return false;
    }

    bool loadImageDimensions(const std::shared_ptr<const IParametersHandler>& ptr,
                             const std::string& widthParameter,
                             const std::string& heightParameter,
                             const std::vector<std::string>& cameraNames,
                             std::unordered_map<std::string, Dimensions>& dimensions)
    {
        constexpr std::string_view logPrefix = "[DinrailCameraBridge::Impl::loadImageDimensions]";

        std::vector<int> width;
        std::vector<int> height;
        if (!ptr->getParameter(widthParameter, width)
            || !ptr->getParameter(heightParameter, height))
        {
            log()->info("{} '{}' and / or '{}' are not provided. The image size will be retrieved "
                        "from the intrinsic parameters of the camera.",
                        logPrefix,
                        widthParameter,
                        heightParameter);
            return true;
        }

        if (width.size() != cameraNames.size() || height.size() != cameraNames.size())
        {
            log()->error("{} The size of '{}' and '{}' must match the number of cameras.",
                         logPrefix,
                         widthParameter,
                         heightParameter);
            return false;
        }

        for (std::size_t idx = 0; idx < cameraNames.size(); idx++)
        {
            if (width[idx] <= 0 || height[idx] <= 0)
            {
                log()->error("{} Invalid image size {}x{} for the camera named {}. The width and "
                             "the height must be positive.",
                             logPrefix,
                             width[idx],
                             height[idx],
                             cameraNames[idx]);
                return false;
            }
            dimensions[cameraNames[idx]]
                = {static_cast<std::size_t>(width[idx]), static_cast<std::size_t>(height[idx])};
        }
        return true;
    }

    bool configureCameras(const std::shared_ptr<const IParametersHandler>& ptr)
    {
        constexpr std::string_view logPrefix = "[DinrailCameraBridge::Impl::configureCameras]";

        if (std::vector<std::string> rgbCameras; ptr->getParameter("rgb_cameras_list", rgbCameras))
        {
            log()->error("{} 'rgb_cameras_list' is not supported, as dinrail does not provide an "
                         "interface for RGB-only cameras. Use 'rgbd_cameras_list'.",
                         logPrefix);
            return false;
        }

        if (!ptr->getParameter("rgbd_cameras_list", metaData.sensorsList.rgbdCamerasList))
        {
            log()->warn("{} 'rgbd_cameras_list' is not provided. No camera is configured.",
                        logPrefix);
            return true;
        }

        metaData.bridgeOptions.isRGBDCameraEnabled = true;
        return loadImageDimensions(ptr,
                                   "rgbd_image_width",
                                   "rgbd_image_height",
                                   metaData.sensorsList.rgbdCamerasList,
                                   metaData.bridgeOptions.rgbdImgDimensions)
               && loadImageDimensions(ptr,
                                      "rgbd_depth_image_width",
                                      "rgbd_depth_image_height",
                                      metaData.sensorsList.rgbdCamerasList,
                                      metaData.bridgeOptions.depthImgDimensions);
    }
};

DinrailCameraBridge::DinrailCameraBridge()
    : m_pimpl(std::make_unique<Impl>())
{
}

DinrailCameraBridge::~DinrailCameraBridge() = default;

bool DinrailCameraBridge::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[DinrailCameraBridge::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The handler is not pointing to an already initialized memory.", logPrefix);
        return false;
    }

    *m_pimpl = Impl();

    bool streamCameras{false};
    if (ptr->getParameter("stream_cameras", streamCameras) && streamCameras)
    {
        auto camerasHandler = ptr->getGroup("Cameras").lock();
        if (camerasHandler == nullptr || !m_pimpl->configureCameras(camerasHandler))
        {
            log()->error("{} The 'Cameras' group could not be initialized from the configuration.",
                         logPrefix);
            return false;
        }
    }

    m_pimpl->bridgeInitialized = true;
    return true;
}

bool DinrailCameraBridge::setDevicesList(const std::vector<DinrailDeviceDescriptor>& devices)
{
    constexpr auto logPrefix = "[DinrailCameraBridge::setDevicesList]";

    if (!m_pimpl->bridgeInitialized)
    {
        log()->error("{} Please initialize DinrailCameraBridge before calling setDevicesList().",
                     logPrefix);
        return false;
    }

    m_pimpl->devicesAttached = false;
    m_pimpl->cameras.clear();
    auto& options = m_pimpl->metaData.bridgeOptions;

    for (const auto& cameraName : m_pimpl->metaData.sensorsList.rgbdCamerasList)
    {
        const auto descriptor = std::find_if(devices.begin(), devices.end(), [&](const auto& d) {
            return d.key == cameraName;
        });
        if (descriptor == devices.end() || !descriptor->isValid())
        {
            log()->error("{} Unable to find an open device named {}.", logPrefix, cameraName);
            m_pimpl->cameras.clear();
            return false;
        }

        Impl::Camera camera;
        camera.device = descriptor->device;
        if (!camera.device->view(camera.sensor) || camera.sensor == nullptr)
        {
            log()->error("{} The device named {} does not implement dinrail::IRGBDSensor.",
                         logPrefix,
                         cameraName);
            m_pimpl->cameras.clear();
            return false;
        }

        // The sizes not configured by the user are taken from the intrinsic parameters. They are
        // missing for the color images of the cameras that stream only the depth.
        dinrail::CameraIntrinsics intrinsics;
        if (options.rgbdImgDimensions.find(cameraName) == options.rgbdImgDimensions.end())
        {
            options.rgbdImgDimensions[cameraName]
                = camera.sensor->getRgbIntrinsics(intrinsics).ok()
                      ? Dimensions{intrinsics.width, intrinsics.height}
                      : Dimensions{0, 0};
        }
        if (options.depthImgDimensions.find(cameraName) == options.depthImgDimensions.end())
        {
            options.depthImgDimensions[cameraName]
                = camera.sensor->getDepthIntrinsics(intrinsics).ok()
                      ? Dimensions{intrinsics.width, intrinsics.height}
                      : Dimensions{0, 0};
        }

        // Preallocate the buffers, so that reading the images does not allocate memory.
        const auto& [rgbWidth, rgbHeight] = options.rgbdImgDimensions[cameraName];
        const auto& [depthWidth, depthHeight] = options.depthImgDimensions[cameraName];
        camera.rgb.data.reserve(rgbWidth * rgbHeight * 3);
        camera.bgr.create(static_cast<int>(rgbHeight), static_cast<int>(rgbWidth), CV_8UC3);
        camera.depth.data.reserve(depthWidth * depthHeight);

        m_pimpl->cameras.emplace(cameraName, std::move(camera));
    }

    m_pimpl->devicesAttached = true;
    return true;
}

bool DinrailCameraBridge::isValid() const
{
    return m_pimpl->checkValid("[DinrailCameraBridge::isValid]");
}

const CameraBridgeMetaData& DinrailCameraBridge::getMetaData() const
{
    return m_pimpl->metaData;
}

bool DinrailCameraBridge::getRGBCamerasList(std::vector<std::string>& rgbCamerasList)
{
    if (!m_pimpl->checkValid("[DinrailCameraBridge::getRGBCamerasList]"))
    {
        return false;
    }
    rgbCamerasList = m_pimpl->metaData.sensorsList.rgbCamerasList;
    return true;
}

bool DinrailCameraBridge::getRGBDCamerasList(std::vector<std::string>& rgbdCamerasList)
{
    if (!m_pimpl->checkValid("[DinrailCameraBridge::getRGBDCamerasList]"))
    {
        return false;
    }
    rgbdCamerasList = m_pimpl->metaData.sensorsList.rgbdCamerasList;
    return true;
}

bool DinrailCameraBridge::getColorImage(
    const std::string& camName,
    cv::Mat& colorImg,
    std::optional<std::reference_wrapper<std::chrono::nanoseconds>> receiveTime)
{
    constexpr auto logPrefix = "[DinrailCameraBridge::getColorImage]";
    if (!m_pimpl->checkValid(logPrefix))
    {
        return false;
    }

    Impl::Camera* camera = m_pimpl->findCamera(logPrefix, camName);
    if (camera == nullptr
        || !Impl::checkReadStatus(logPrefix, camName, camera->sensor->getRgbImage(camera->rgb)))
    {
        return false;
    }

    auto& image = camera->rgb;
    const cv::Mat source(static_cast<int>(image.height),
                         static_cast<int>(image.width),
                         CV_8UC3,
                         image.data.data());
    if (image.format == dinrail::PixelFormat::Rgb8)
    {
        cv::cvtColor(source, camera->bgr, cv::COLOR_RGB2BGR);
        colorImg = camera->bgr;
    } else if (image.format == dinrail::PixelFormat::Bgr8)
    {
        colorImg = source;
    } else
    {
        log()->error("{} Unable to convert the image of the camera named {} into OpenCV. Only "
                     "the Rgb8 and Bgr8 formats are supported.",
                     logPrefix,
                     camName);
        return false;
    }

    if (colorImg.rows <= 0 || colorImg.cols <= 0)
    {
        log()->error("{} {} image with invalid size.", logPrefix, camName);
        return false;
    }

    updateReceiveTime(image.stamp, camera->rgbStamp, camera->rgbTime);
    setReceiveTime(receiveTime, camera->rgbTime);
    return true;
}

bool DinrailCameraBridge::getDepthImage(
    const std::string& camName,
    cv::Mat& depthImg,
    std::optional<std::reference_wrapper<std::chrono::nanoseconds>> receiveTime)
{
    constexpr auto logPrefix = "[DinrailCameraBridge::getDepthImage]";
    if (!m_pimpl->checkValid(logPrefix))
    {
        return false;
    }

    Impl::Camera* camera = m_pimpl->findCamera(logPrefix, camName);
    if (camera == nullptr
        || !Impl::checkReadStatus(logPrefix, camName, camera->sensor->getDepthImage(camera->depth)))
    {
        return false;
    }

    auto& image = camera->depth;
    depthImg = cv::Mat(static_cast<int>(image.height),
                       static_cast<int>(image.width),
                       CV_32FC1,
                       image.data.data());
    if (depthImg.rows <= 0 || depthImg.cols <= 0)
    {
        log()->error("{} {} image with invalid size.", logPrefix, camName);
        return false;
    }

    updateReceiveTime(image.stamp, camera->depthStamp, camera->depthTime);
    setReceiveTime(receiveTime, camera->depthTime);
    return true;
}
