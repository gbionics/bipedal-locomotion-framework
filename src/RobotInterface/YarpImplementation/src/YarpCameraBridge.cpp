/**
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotInterface/YarpCameraBridge.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

// YARP sig
#include <yarp/os/Stamp.h>
#include <yarp/sig/Image.h>

// YARP Camera Interfaces
#include <yarp/cv/Cv.h>
#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/dev/IRGBDSensor.h>

// std
#include <chrono>
#include <string_view>
#include <type_traits>

using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::GenericContainer;
using namespace BipedalLocomotion::ParametersHandler;

namespace
{
void setReceiveTime(std::optional<std::reference_wrapper<std::chrono::nanoseconds>>& receiveTime,
                    const std::chrono::nanoseconds& time)
{
    if (receiveTime)
    {
        receiveTime->get() = time;
    }
}

bool isValidDimension(const std::pair<std::size_t, std::size_t>& dimensions)
{
    return dimensions.first > 0 && dimensions.second > 0;
}

/**
 * Store the envelope of the image just read. The receive time is updated only for a new frame: the
 * YARP clients return the last frame received until a new one arrives.
 */
void updateReceiveTime(const yarp::os::Stamp& newStamp,
                       yarp::os::Stamp& stamp,
                       std::chrono::nanoseconds& time)
{
    const bool isSameFrame = newStamp.isValid() && stamp.isValid()
                             && newStamp.getCount() == stamp.getCount()
                             && newStamp.getTime() == stamp.getTime();
    if (!isSameFrame)
    {
        time = BipedalLocomotion::clock().now();
    }
    stamp = newStamp;
}
} // namespace

struct YarpCameraBridge::Impl
{
    template <typename PixelCode, typename CameraType> struct StampedYARPImage
    {
        yarp::sig::ImageOf<PixelCode> image;
        CameraType* interface{nullptr};
        yarp::os::Stamp stamp;
        std::chrono::nanoseconds time{0};

        /**
         * Resize image buffers
         */
        void resize(const std::pair<std::size_t, std::size_t>& dimensions)
        {
            if (image.width() != dimensions.first || image.height() != dimensions.second)
            {
                image.resize(dimensions.first, dimensions.second);
            }
        }

        bool readCameraImage()
        {
            bool ok{false};
            // The frame grabber interface does not provide the envelope, it stays invalid.
            yarp::os::Stamp newStamp;
            if constexpr (std::is_same_v<CameraType, yarp::dev::IFrameGrabberImage>)
            {
                static_assert(std::is_same_v<PixelCode, yarp::sig::PixelRgb>,
                              "Frame grabbers handle only RGB images.");
                ok = interface->getImage(image);
            } else
            {
                static_assert(std::is_same_v<CameraType, yarp::dev::IRGBDSensor>
                                  && std::is_same_v<PixelCode, yarp::sig::PixelFloat>,
                              "RGBD sensors provide depth images as PixelFloat.");
                ok = interface->getDepthImage(image, &newStamp);
            }

            if (!ok)
            {
                return false;
            }

            updateReceiveTime(newStamp, stamp, time);
            return true;
        }
    };

    struct StampedYARPFlexImage
    {
        yarp::sig::FlexImage image;
        yarp::dev::IRGBDSensor* interface{nullptr};
        yarp::os::Stamp stamp;
        std::chrono::nanoseconds time{0};

        /**
         * Resize image buffers
         */
        void resize(const std::pair<std::size_t, std::size_t>& dimensions)
        {
            image.setPixelCode(VOCAB_PIXEL_RGB);
            if (image.width() != dimensions.first || image.height() != dimensions.second)
            {
                image.resize(dimensions.first, dimensions.second);
            }
        }

        bool readCameraImage()
        {
            yarp::os::Stamp newStamp;
            if (!interface->getRgbImage(image, &newStamp))
            {
                return false;
            }

            updateReceiveTime(newStamp, stamp, time);
            return true;
        }
    };

    std::unordered_map<std::string, yarp::dev::IFrameGrabberImage*>
        wholeBodyFrameGrabberInterface; /** < map of cameras attached through frame grabber
                                           interfaces */
    std::unordered_map<std::string, yarp::dev::IRGBDSensor*> wholeBodyRGBDInterface; /** < map of
                                                                                        cameras
                                                                                        attached
                                                                                        through RGBD
                                                                                        interfaces
                                                                                      */
    CameraBridgeMetaData metaData; /**< struct holding meta data **/
    bool bridgeInitialized{false}; /**< flag set to true if the bridge is successfully initialized
                                    */
    bool driversAttached{false}; /**< flag set to true if the bridge is successfully attached to
                                    required device drivers */

    std::unordered_map<std::string,
                       StampedYARPImage<yarp::sig::PixelFloat, yarp::dev::IRGBDSensor>>
        depthImages;
    std::unordered_map<std::string, StampedYARPFlexImage> flexImages;
    std::unordered_map<std::string,
                       StampedYARPImage<yarp::sig::PixelRgb, yarp::dev::IFrameGrabberImage>>
        rgbImages;

    /**
     * Check if sensor is available in the relevant sensor map
     */
    template <typename SensorType>
    bool checkSensor(const std::unordered_map<std::string, SensorType*>& sensorMap,
                     const std::string& sensorName)
    {
        if (sensorMap.find(sensorName) == sensorMap.end())
        {
            return false;
        }
        return true;
    }

    /**
     * Check if sensor is available in the relevant sensor measurement map
     */
    template <typename YARPDataType>
    bool checkValidSensorMeasure(std::string_view logPrefix,
                                 const std::unordered_map<std::string, YARPDataType>& sensorMap,
                                 const std::string& sensorName)
    {
        if (!checkValid(logPrefix))
        {
            return false;
        }

        if (sensorMap.find(sensorName) == sensorMap.end())
        {
            log()->error("{} {} sensor unavailable in the measurement map.", logPrefix, sensorName);
            return false;
        }

        return true;
    }

    /**
     * Check if the bridge is successfully initialized and attached to required device drivers
     */
    bool checkValid(const std::string_view methodName)
    {
        if (!(bridgeInitialized && driversAttached))
        {
            log()->error("{} CameraBridge is not ready. Please initialize and set drivers list.",
                         methodName);
            return false;
        }
        return true;
    }

    using SubConfigLoader = bool (YarpCameraBridge::Impl::*)(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>,
        CameraBridgeMetaData&);
    /**
     * Checks is a stream is enabled in configuration and
     * loads the relevant stream group from configuration
     */
    bool subConfigLoader(
        const std::string& enableStreamString,
        const std::string& streamGroupString,
        const SubConfigLoader loader,
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
        CameraBridgeMetaData& metaData,
        bool& enableStreamFlag)
    {
        constexpr auto logPrefix = "[YarpCameraBridge::Impl::subConfigLoader]";

        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            log()->error("{} The handler is not pointing to an already initialized memory.",
                         logPrefix);
            return false;
        }

        enableStreamFlag = false;
        if (ptr->getParameter(enableStreamString, enableStreamFlag) && enableStreamFlag)
        {
            auto groupHandler = ptr->getGroup(streamGroupString);
            if (!(this->*loader)(groupHandler, metaData))
            {
                log()->error("{} {} group could not be initialized from the configuration file.",
                             logPrefix,
                             streamGroupString);

                return false;
            }
        }

        return true;
    }

    /**
     * Load optional image dimensions for a camera list
     */
    bool loadImageDimensions(
        const std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>& ptr,
        const std::string& widthParameter,
        const std::string& heightParameter,
        const std::vector<std::string>& cameras,
        std::unordered_map<std::string, std::pair<std::size_t, std::size_t>>& dimensions)
    {
        constexpr std::string_view logPrefix = "[YarpCameraBridge::Impl::loadImageDimensions]";

        std::vector<int> width, height;
        if (!ptr->getParameter(widthParameter, width)
            || !ptr->getParameter(heightParameter, height))
        {
            log()->info("{} '{}' and / or '{}' are not provided. The image size will be "
                        "retrieved from the YARP interface.",
                        logPrefix,
                        widthParameter,
                        heightParameter);
            return true;
        }

        if (width.size() != cameras.size() || height.size() != cameras.size())
        {
            log()->error("{} The size of '{}' and '{}' must match the number of cameras.",
                         logPrefix,
                         widthParameter,
                         heightParameter);
            return false;
        }

        for (std::size_t idx = 0; idx < cameras.size(); idx++)
        {
            if (width[idx] <= 0 || height[idx] <= 0)
            {
                log()->error("{} Invalid image size {}x{} for the camera named {}. The width and "
                             "the height must be positive.",
                             logPrefix,
                             width[idx],
                             height[idx],
                             cameras[idx]);
                return false;
            }

            dimensions[cameras[idx]] = {static_cast<std::size_t>(width[idx]),
                                        static_cast<std::size_t>(height[idx])};
        }

        return true;
    }

    /**
     * Configure cameras meta data
     */
    bool configureCameras(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
        CameraBridgeMetaData& metaData)
    {
        constexpr std::string_view logPrefix = "[YarpCameraBridge::Impl::configureCameras]";
        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            return false;
        }

        if (ptr->getParameter("rgb_cameras_list", metaData.sensorsList.rgbCamerasList))
        {
            metaData.bridgeOptions.isRGBCameraEnabled = true;
            if (!loadImageDimensions(ptr,
                                     "rgb_image_width",
                                     "rgb_image_height",
                                     metaData.sensorsList.rgbCamerasList,
                                     metaData.bridgeOptions.rgbImgDimensions))
            {
                return false;
            }
        }

        if (ptr->getParameter("rgbd_cameras_list", metaData.sensorsList.rgbdCamerasList))
        {
            metaData.bridgeOptions.isRGBDCameraEnabled = true;
            if (!loadImageDimensions(ptr,
                                     "rgbd_image_width",
                                     "rgbd_image_height",
                                     metaData.sensorsList.rgbdCamerasList,
                                     metaData.bridgeOptions.rgbdImgDimensions)
                || !loadImageDimensions(ptr,
                                        "rgbd_depth_image_width",
                                        "rgbd_depth_image_height",
                                        metaData.sensorsList.rgbdCamerasList,
                                        metaData.bridgeOptions.depthImgDimensions))
            {
                return false;
            }
        }

        if (!metaData.bridgeOptions.isRGBCameraEnabled
            && !metaData.bridgeOptions.isRGBDCameraEnabled)
        {
            log()->warn("{} None of the camera types configured. Cannot use Camera bridge.",
                        logPrefix);
            return true;
        }

        return true;
    }

    /**
     * Attach cameras
     */
    template <typename CameraType>
    bool attachCamera(const yarp::dev::PolyDriverList& devList,
                      const std::string sensorName,
                      std::unordered_map<std::string, CameraType*>& sensorMap)
    {
        constexpr std::string_view logPrefix = "[YarpCameraBridge::Impl::attachCamera] ";
        for (int devIdx = 0; devIdx < devList.size(); devIdx++)
        {
            if (sensorName != devList[devIdx]->key)
            {
                continue;
            }

            CameraType* cameraInterface{nullptr};
            if (devList[devIdx]->poly->view(cameraInterface))
            {
                if (cameraInterface == nullptr)
                {
                    log()->error("{} Could not view interface.", logPrefix);
                    return false;
                }
                sensorMap[devList[devIdx]->key] = cameraInterface;
            }
        }
        return true;
    }

    /**
     * Attach all cameras
     */
    bool attachAllCameras(const yarp::dev::PolyDriverList& devList)
    {
        constexpr auto prefix = "[YarpCameraBridge::attachAllCameras]";

        if (!metaData.bridgeOptions.isRGBCameraEnabled
            && !metaData.bridgeOptions.isRGBDCameraEnabled)
        {
            // do nothing
            log()->warn("{} No camera types enable. Not attaching any cameras.", prefix);
            return true;
        }

        if (metaData.bridgeOptions.isRGBCameraEnabled)
        {
            constexpr auto interfaceType = "RGB Cameras";
            if (!attachAllCamerasOfSpecificType(devList,
                                                metaData.sensorsList.rgbCamerasList,
                                                interfaceType,
                                                wholeBodyFrameGrabberInterface))
            {
                return false;
            }
        }

        if (metaData.bridgeOptions.isRGBDCameraEnabled)
        {
            constexpr auto interfaceTypeDepth = "RGBD Cameras";
            if (!attachAllCamerasOfSpecificType(devList,
                                                metaData.sensorsList.rgbdCamerasList,
                                                interfaceTypeDepth,
                                                wholeBodyRGBDInterface))
            {
                return false;
            }
        }

        return true;
    }

    /**
     * Attach all cameras of specific type and resize image buffers
     */
    template <typename CameraType>
    bool attachAllCamerasOfSpecificType(const yarp::dev::PolyDriverList& devList,
                                        const std::vector<std::string>& camList,
                                        std::string_view interfaceType,
                                        std::unordered_map<std::string, CameraType*>& sensorMap)
    {
        constexpr auto logPrefix = "[YarpCameraBridge::Impl::attachAllCamerasOfSpecificType]";
        for (const auto& cam : camList)
        {
            if (!attachCamera(devList, cam, sensorMap))
            {
                return false;
            }
        }

        if (sensorMap.size() != camList.size())
        {
            log()->error("{} could not attach all desired cameras of type {}.",
                         logPrefix,
                         interfaceType);
            return false;
        }

        return true;
    }

}; // end YarpCameraBridge::Impl

YarpCameraBridge::YarpCameraBridge()
    : m_pimpl(std::make_unique<Impl>())
{
}

YarpCameraBridge::~YarpCameraBridge() = default;

bool YarpCameraBridge::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[YarpCameraBridge::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The handler is not pointing to an already initialized memory.", logPrefix);
        return false;
    }

    m_pimpl->bridgeInitialized = false;
    m_pimpl->driversAttached = false;
    m_pimpl->metaData = {};
    m_pimpl->wholeBodyFrameGrabberInterface.clear();
    m_pimpl->wholeBodyRGBDInterface.clear();
    m_pimpl->rgbImages.clear();
    m_pimpl->flexImages.clear();
    m_pimpl->depthImages.clear();

    bool ret{true};
    bool useCameras{false};
    ret = m_pimpl->subConfigLoader("stream_cameras",
                                   "Cameras",
                                   &YarpCameraBridge::Impl::configureCameras,
                                   handler,
                                   m_pimpl->metaData,
                                   useCameras);

    if (!ret)
    {
        log()->error("{} Unable to configure the cameras.", logPrefix);
        return false;
    }

    m_pimpl->bridgeInitialized = true;
    return true;
}

bool YarpCameraBridge::setDriversList(const yarp::dev::PolyDriverList& deviceDriversList)
{
    constexpr std::string_view logPrefix = "[YarpCameraBridge::setDriversList] ";

    if (!m_pimpl->bridgeInitialized)
    {
        log()->error("{} Please initialize YarpCameraBridge before calling setDriversList().",
                     logPrefix);
        return false;
    }

    bool ret{true};
    ret = ret && m_pimpl->attachAllCameras(deviceDriversList);

    if (!ret)
    {
        log()->error("{} Failed to attach to one or more device drivers.", logPrefix);
        return false;
    }

    auto& options = m_pimpl->metaData.bridgeOptions;

    // in the case the user does not provide the images size the YarpCameraBridge will be retrieve
    // it from the yarp interface
    if (options.rgbImgDimensions.empty())
    {
        yarp::sig::ImageOf<yarp::sig::PixelRgb> tmpImg;
        for (const auto& [cameraName, interface] : m_pimpl->wholeBodyFrameGrabberInterface)
        {
            constexpr std::size_t maxAttempt = 20;
            std::size_t attempt = 0;

            // for each camera we wait for the first image. It is necessary to get the image size
            while (!interface->getImage(tmpImg))
            {
                attempt++;
                if (attempt > maxAttempt)
                {
                    log()->error("{} Unable to get the image from the camera named {}.",
                                 logPrefix,
                                 cameraName);
                    return false;
                }
                using namespace std::chrono_literals;
                BipedalLocomotion::clock().yield();
                BipedalLocomotion::clock().sleepFor(10ms);
            }

            options.rgbImgDimensions[cameraName] = {tmpImg.width(), tmpImg.height()};
        }
    }

    auto getInterfaceDimensions = [](int width, int height) {
        if (width <= 0 || height <= 0)
        {
            return std::pair<std::size_t, std::size_t>{0, 0};
        }
        return std::pair<std::size_t, std::size_t>{static_cast<std::size_t>(width),
                                                   static_cast<std::size_t>(height)};
    };

    for (const auto& [cameraName, interface] : m_pimpl->wholeBodyRGBDInterface)
    {
        if (options.rgbdImgDimensions.find(cameraName) == options.rgbdImgDimensions.end())
        {
            options.rgbdImgDimensions[cameraName]
                = getInterfaceDimensions(interface->getRgbWidth(), interface->getRgbHeight());
        }
        if (options.depthImgDimensions.find(cameraName) == options.depthImgDimensions.end())
        {
            options.depthImgDimensions[cameraName]
                = getInterfaceDimensions(interface->getDepthWidth(), interface->getDepthHeight());
        }
    }

    // Preallocate the image buffers and cache the interface pointers to avoid allocations and map
    // lookups while reading images.
    auto prepareImage = [&logPrefix](const std::string& cameraName,
                                     auto* interface,
                                     const auto& dimensions,
                                     auto& images) {
        auto& image = images[cameraName];
        image.interface = interface;

        const auto dimension = dimensions.find(cameraName);
        if (dimension != dimensions.end() && isValidDimension(dimension->second))
        {
            image.resize(dimension->second);
        } else
        {
            log()->warn("{} Unable to preallocate the image buffer for the camera named {}. "
                        "The buffer will be allocated when the first image is received.",
                        logPrefix,
                        cameraName);
        }
    };

    for (const auto& [cameraName, interface] : m_pimpl->wholeBodyFrameGrabberInterface)
    {
        prepareImage(cameraName, interface, options.rgbImgDimensions, m_pimpl->rgbImages);
    }

    for (const auto& [cameraName, interface] : m_pimpl->wholeBodyRGBDInterface)
    {
        prepareImage(cameraName, interface, options.rgbdImgDimensions, m_pimpl->flexImages);
        prepareImage(cameraName, interface, options.depthImgDimensions, m_pimpl->depthImages);
    }

    m_pimpl->driversAttached = true;
    return true;
}

bool YarpCameraBridge::isValid() const
{
    return m_pimpl->checkValid("[YarpCameraBridge::isValid]");
}

const CameraBridgeMetaData& YarpCameraBridge::getMetaData() const
{
    return m_pimpl->metaData;
}

const CameraBridgeMetaData& YarpCameraBridge::get() const
{
    return this->getMetaData();
}

bool YarpCameraBridge::getRGBCamerasList(std::vector<std::string>& rgbCamerasList)
{
    if (!m_pimpl->checkValid("[YarpCameraBridge::getRGBCamerasList]"))
    {
        return false;
    }
    rgbCamerasList = m_pimpl->metaData.sensorsList.rgbCamerasList;
    return true;
}

bool YarpCameraBridge::getRGBDCamerasList(std::vector<std::string>& rgbdCamerasList)
{
    if (!m_pimpl->checkValid("[YarpCameraBridge::getRGBDCamerasList]"))
    {
        return false;
    }
    rgbdCamerasList = m_pimpl->metaData.sensorsList.rgbdCamerasList;
    return true;
}

bool YarpCameraBridge::getColorImage(
    const std::string& camName,
    cv::Mat& colorImg,
    std::optional<std::reference_wrapper<std::chrono::nanoseconds>> receiveTime)
{
    constexpr auto logPrefix = "[YarpCameraBridge::getColorImage]";
    if (!m_pimpl->checkValid(logPrefix))
    {
        return false;
    }

    std::chrono::nanoseconds imageTime{0};

    // check if the camera name is a standard camera or a depth camera
    auto rgbImage = m_pimpl->rgbImages.find(camName);
    if (rgbImage != m_pimpl->rgbImages.end())
    {
        if (!rgbImage->second.readCameraImage())
        {
            log()->error("{} Unable to read the image from the camera named {}.",
                         logPrefix,
                         camName);
            return false;
        }

        colorImg = yarp::cv::toCvMat(rgbImage->second.image);
        imageTime = rgbImage->second.time;
    } else
    {
        auto flexImage = m_pimpl->flexImages.find(camName);
        if (flexImage == m_pimpl->flexImages.end())
        {
            log()->error("{} Unable to find the camera named {}.", logPrefix, camName);
            return false;
        }

        if (!flexImage->second.readCameraImage())
        {
            log()->error("{} Unable to read the image from the camera named {}.",
                         logPrefix,
                         camName);
            return false;
        }

        auto& yarpImage = flexImage->second.image;
        if (yarpImage.getPixelCode() == VOCAB_PIXEL_BGR)
        {
            colorImg = cv::Mat(yarpImage.height(),
                               yarpImage.width(),
                               yarp::cv::type_code<yarp::sig::PixelBgr>::value,
                               yarpImage.getRawImage(),
                               yarpImage.getRowSize());

            if constexpr (yarp::cv::convert_code_to_cv<yarp::sig::PixelBgr>::value >= 0)
            {
                ::cv::cvtColor(colorImg,
                               colorImg,
                               yarp::cv::convert_code_to_cv<yarp::sig::PixelBgr>::value);
            }
        } else if (yarpImage.getPixelCode() == VOCAB_PIXEL_RGB)
        {
            colorImg = cv::Mat(yarpImage.height(),
                               yarpImage.width(),
                               yarp::cv::type_code<yarp::sig::PixelRgb>::value,
                               yarpImage.getRawImage(),
                               yarpImage.getRowSize());

            if constexpr (yarp::cv::convert_code_to_cv<yarp::sig::PixelRgb>::value >= 0)
            {
                ::cv::cvtColor(colorImg,
                               colorImg,
                               yarp::cv::convert_code_to_cv<yarp::sig::PixelRgb>::value);
            }
        } else
        {
            log()->error("{} Unable to convert the YARP image into OpenCV for the camera named "
                         "{}. Only VOCAB_PIXEL_BGR and VOCAB_PIXEL_RGB are supported.",
                         logPrefix,
                         camName);
            return false;
        }

        imageTime = flexImage->second.time;
    }

    if (colorImg.rows <= 0 || colorImg.cols <= 0)
    {
        log()->error("{} {} image with invalid size.", logPrefix, camName);
        return false;
    }

    setReceiveTime(receiveTime, imageTime);
    return true;
}

bool YarpCameraBridge::getDepthImage(
    const std::string& camName,
    cv::Mat& depthImg,
    std::optional<std::reference_wrapper<std::chrono::nanoseconds>> receiveTime)
{
    constexpr auto logPrefix = "[YarpCameraBridge::getDepthImage]";
    if (!m_pimpl->checkValid(logPrefix))
    {
        return false;
    }

    auto depthImage = m_pimpl->depthImages.find(camName);
    if (depthImage == m_pimpl->depthImages.end())
    {
        log()->error("{} Unable to find the camera named {}.", logPrefix, camName);
        return false;
    }

    if (!depthImage->second.readCameraImage())
    {
        log()->error("{} Unable to read the image from the camera named {}.", logPrefix, camName);
        return false;
    }

    depthImg = yarp::cv::toCvMat(depthImage->second.image);
    if (depthImg.rows <= 0 || depthImg.cols <= 0)
    {
        log()->error("{} {} image with invalid size.", logPrefix, camName);
        return false;
    }

    setReceiveTime(receiveTime, depthImage->second.time);
    return true;
}
