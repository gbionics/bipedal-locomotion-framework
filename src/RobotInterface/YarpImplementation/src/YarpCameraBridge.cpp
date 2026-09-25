/**
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotInterface/YarpCameraBridge.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

// YARP sig
#include <yarp/sig/Image.h>

// YARP Camera Interfaces
#include <yarp/cv/Cv.h>
#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/dev/IRGBDSensor.h>

// std
#include <algorithm>
#include <cmath>
#include <set>

using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::GenericContainer;
using namespace BipedalLocomotion::ParametersHandler;

struct YarpCameraBridge::Impl
{
    template <typename PixelCode> struct StampedYARPImage
    {
        yarp::sig::ImageOf<PixelCode> image;
        double time;

        StampedYARPImage() = default;

        StampedYARPImage(std::size_t width, std::size_t height)
        {
            this->resize({width, height});
        };

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

        template <typename CameraType>
        bool readCameraImage(const std::string& cameraName, CameraType* interface)
        {
            bool ok{true};
            constexpr auto logPrefix = "[StampedYARPImage::readCameraImage]";

            // StampedYARPImage is defined as a pair of yarp::sig::image and double
            yarp::os::Stamp* txTimestamp{nullptr};
            if constexpr (std::is_same_v<CameraType, yarp::dev::IFrameGrabberImage>)
            {
                // (imageType != "RGB")
                if constexpr (!std::is_same_v<PixelCode, yarp::sig::PixelRgb>)
                {
                    log()->error("{} Frame Grabber {} handles only RGB image.",
                                 logPrefix,
                                 cameraName);
                    return false;
                }
                ok = interface->getImage(image);
            } else if constexpr (std::is_same_v<CameraType, yarp::dev::IRGBDSensor>)
            {
                // (imageType != "DEPTH")
                if constexpr (std::is_same_v<PixelCode, yarp::sig::PixelFloat>)
                {
                    ok = interface->getDepthImage(image, txTimestamp);
                }
            }

            if (!ok)
            {
                log()->error("{} Unable to read from {}, use previous image.",
                             logPrefix,
                             cameraName);
                return false;
            }

            time = std::chrono::duration<double>(BipedalLocomotion::clock().now()).count();
            return true;
        }
    };

    struct StampedYARPFlexImage
    {
        yarp::sig::FlexImage image;
        double time;

        StampedYARPFlexImage() = default;

        StampedYARPFlexImage(std::size_t width, std::size_t height)
        {
            this->resize({width, height});
        };

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

        template <typename CameraType>
        bool readCameraImage(const std::string& cameraName, CameraType* interface)
        {
            bool ok{true};
            constexpr auto logPrefix = "[StampedYARPFlexImage::readCameraImage]";

            yarp::os::Stamp* txTimestamp{nullptr};
            if constexpr (std::is_same_v<CameraType, yarp::dev::IRGBDSensor>)
            {
                ok = interface->getRgbImage(image, txTimestamp);
            }

            if (!ok)
            {
                log()->error("{} Unable to read from {}, use previous image.",
                             logPrefix,
                             cameraName);
                return false;
            }

            time = std::chrono::duration<double>(BipedalLocomotion::clock().now()).count();
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

    std::unordered_map<std::string, StampedYARPImage<yarp::sig::PixelFloat>> depthImages;
    std::unordered_map<std::string, StampedYARPFlexImage> flexImages;
    std::unordered_map<std::string, StampedYARPImage<yarp::sig::PixelRgb>> rgbImages;

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
     * Configure cameras meta data
     */
    bool configureCameras(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
        CameraBridgeMetaData& metaData)
    {
        constexpr std::string_view logPrefix = "[YarpCameraBridge::Impl::configureCameras] ";
        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            return false;
        }

        if (ptr->getParameter("rgb_cameras_list", metaData.sensorsList.rgbCamerasList))
        {
            metaData.bridgeOptions.isRGBCameraEnabled = true;

            std::vector<int> rgbWidth, rgbHeight;
            bool ok = ptr->getParameter("rgb_image_width", rgbWidth);
            ok = ok && ptr->getParameter("rgb_image_height", rgbHeight);

            if (ok)
            {
                if ((rgbWidth.size() != metaData.sensorsList.rgbCamerasList.size())
                    || (rgbHeight.size() != metaData.sensorsList.rgbCamerasList.size()))
                {
                    log()->error("{} Parameters list size mismatch.", logPrefix);
                    return false;
                }

                for (int idx = 0; idx < rgbHeight.size(); idx++)
                {
                    std::pair<int, int> imgDimensions(rgbWidth[idx], rgbHeight[idx]);
                    auto cameraName{metaData.sensorsList.rgbCamerasList[idx]};
                    metaData.bridgeOptions.rgbImgDimensions[cameraName] = imgDimensions;
                }
            } else
            {
                log()->info("{} 'rgb_image_width' and / or 'rgb_image_height' are not provided. "
                            "The image size will be retrieved from the YARP interface.",
                            logPrefix);
            }
        }

        if (ptr->getParameter("rgbd_cameras_list", metaData.sensorsList.rgbdCamerasList))
        {
            metaData.bridgeOptions.isRGBDCameraEnabled = true;

            std::vector<int> rgbdCamWidth, rgbdCamHeight;
            bool ok = ptr->getParameter("rgbd_image_width", rgbdCamWidth);
            ok = ok && ptr->getParameter("rgbd_image_height", rgbdCamHeight);

            if (ok)
            {
                if ((rgbdCamWidth.size() != metaData.sensorsList.rgbdCamerasList.size())
                    || (rgbdCamHeight.size() != metaData.sensorsList.rgbdCamerasList.size()))
                {
                    log()->error("{} Parameters list size mismatch.", logPrefix);
                    return false;
                }

                for (int idx = 0; idx < rgbdCamHeight.size(); idx++)
                {
                    std::pair<int, int> imgDimensions(rgbdCamWidth[idx], rgbdCamHeight[idx]);
                    auto cameraName{metaData.sensorsList.rgbdCamerasList[idx]};
                    metaData.bridgeOptions.rgbdImgDimensions[cameraName] = imgDimensions;
                }
            } else
            {
                log()->info("{} 'rgb_image_width' and / or 'rgb_image_height' are not provided. "
                            "The image size will be retrieved from the YARP interface.",
                            logPrefix);
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
        log()->error("{} Skipping the configuration of Cameras. YarpCameraBridge will not stream "
                     "relevant measures.",
                     logPrefix);
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

    // in the case the user does not provide the images size the YarpCameraBridge will be retrieve
    // it from the yarp interface
    if (m_pimpl->metaData.bridgeOptions.rgbImgDimensions.empty())
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

            m_pimpl->metaData.bridgeOptions.rgbImgDimensions[cameraName]
                = {interface->width(), interface->height()};
        }
    }

    if (m_pimpl->metaData.bridgeOptions.rgbdImgDimensions.empty())
    {
        for (const auto& [cameraName, interface] : m_pimpl->wholeBodyRGBDInterface)
        {
            if ((interface->getRgbWidth() != interface->getDepthWidth())
                || (interface->getRgbHeight() != interface->getDepthHeight()))
            {
                log()->error("{} Mismatch between the rgb and depth width or the rgb height and "
                             "the depth height. For the camera {}. YarpCameraBridge do not support "
                             "cameras having a different resolutions. The support will be added in "
                             "the future.",
                             logPrefix,
                             cameraName);
                return false;
            }
            m_pimpl->metaData.bridgeOptions.rgbdImgDimensions[cameraName]
                = {interface->getRgbWidth(), interface->getRgbHeight()};
        }
    }

    // Initialize the images container accordingly to the metadata this is required to make
    // YarpCameraBridge thread safe
    // This should speedup also the getColorImage function since the memory is already allocated
    for (const auto& [cameraName, dimension] : m_pimpl->metaData.bridgeOptions.rgbImgDimensions)
    {
        m_pimpl->rgbImages[cameraName].resize(dimension);
    }

    for (const auto& [cameraName, dimension] : m_pimpl->metaData.bridgeOptions.rgbdImgDimensions)
    {
        m_pimpl->flexImages[cameraName].resize(dimension);
        m_pimpl->depthImages[cameraName].resize(dimension);
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
    std::optional<std::reference_wrapper<double>> receiveTimeInSeconds)
{
    if (!m_pimpl->checkValid("YarpCameraBridge::getColorImage "))
    {
        return false;
    }

    // check if the camera name is a standard camera or a depth camera
    auto rgbImage = m_pimpl->rgbImages.find(camName);
    if (rgbImage != m_pimpl->rgbImages.end())
    {
        //
        if (!rgbImage->second.readCameraImage(camName,
                                              m_pimpl->wholeBodyFrameGrabberInterface.at(camName)))
        {
            log()->error("[YarpCameraBridge::getColorImage] {} could not read image.", camName);
            return false;
        }

        colorImg = yarp::cv::toCvMat(rgbImage->second.image);
        if (receiveTimeInSeconds)
        {
            receiveTimeInSeconds.value().get() = rgbImage->second.time;
        }
    } else
    {
        auto flexImage = m_pimpl->flexImages.find(camName);
        if (flexImage == m_pimpl->flexImages.end())
        {
            log()->error("[YarpCameraBridge::getColorImage] Unable to find the camera named {}.",
                         camName);
            return false;
        }

        if (!flexImage->second.readCameraImage(camName,
                                               m_pimpl->wholeBodyRGBDInterface.at(camName)))
        {
            log()->error("[YarpCameraBridge::getColorImage] {} could not read image.", camName);
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

            if (yarp::cv::convert_code_to_cv<yarp::sig::PixelBgr>::value >= 0)
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

            if (yarp::cv::convert_code_to_cv<yarp::sig::PixelRgb>::value >= 0)
            {
                ::cv::cvtColor(colorImg,
                               colorImg,
                               yarp::cv::convert_code_to_cv<yarp::sig::PixelRgb>::value);
            }
        } else
        {
            log()->error("[YarpCameraBridge::getColorImage] Unable to convert the yarp image into "
                         "opencv for the camera named: {}. Only VOCAB_PIXEL_BGR and "
                         "VOCAB_PIXEL_RGB are supported.",
                         camName);
            return false;
        }

        if (receiveTimeInSeconds)
        {
            receiveTimeInSeconds.value().get() = flexImage->second.time;
        }
    }

    if (colorImg.rows <= 0 || colorImg.cols <= 0)
    {
        log()->error("[YarpCameraBridge::getColorImage] {} image with invalid size.", camName);
        return false;
    }

    return true;
}

bool YarpCameraBridge::getDepthImage(
    const std::string& camName,
    cv::Mat& depthImg,
    std::optional<std::reference_wrapper<double>> receiveTimeInSeconds)
{

    constexpr auto prefix = "[YarpCameraBridge::getDepthImage]";
    if (!m_pimpl->checkValid(prefix))
    {
        return false;
    }

    // check if the camera name is a standard camera or a depth camera
    auto depthImage = m_pimpl->depthImages.find(camName);
    if (depthImage == m_pimpl->depthImages.end())
    {
        log()->error("{} Unable to find the camera named {}.", prefix, camName);
        return false;
    }

    if (!depthImage->second.readCameraImage(camName, m_pimpl->wholeBodyRGBDInterface.at(camName)))
    {
        log()->error("{} {} could not read image.", prefix, camName);
        return false;
    }

    depthImg = yarp::cv::toCvMat(depthImage->second.image);
    if (receiveTimeInSeconds)
    {
        receiveTimeInSeconds.value().get() = depthImage->second.time;
    }

    if (depthImg.rows <= 0 || depthImg.cols <= 0)
    {
        log()->error("{}  {} image with invalid size.", prefix, camName);
        return false;
    }

    return true;
}
