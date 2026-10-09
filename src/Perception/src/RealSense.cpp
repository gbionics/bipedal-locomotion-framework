/**
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <algorithm>

#include <BipedalLocomotion/Perception/Capture/RealSense.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <librealsense2/rs.hpp>
#include <tuple>

using namespace BipedalLocomotion::Perception::Capture;

struct RealSense::Impl
{
    bool isValidCamera(const std::string& camName);
    bool startStream();
    void stopStream();

    struct TextureRGB
    {
        int r, g, b;
    };

    /**
     * Copied from Realsense examples
     * see
     * https://github.com/IntelRealSense/librealsense/blob/master/wrappers/pcl/pcl-color/rs-pcl-color.cpp
     */
    TextureRGB rgbTexture(rs2::video_frame textureImg, rs2::texture_coordinate textureXY);
    void toPCL(const rs2::points& points,
               const rs2::video_frame& color,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    bool isIREnabled{false};
    bool isPCLEnabled{false};

    int genericFPS{30};
    BipedalLocomotion::RobotInterface::CameraBridgeMetaData metadata;

    rs2_format colorFormat{RS2_FORMAT_BGR8};
    rs2_format irFormat{RS2_FORMAT_Y8};
    rs2_format depthFormat{RS2_FORMAT_Z16};

    bool isStreaming{false};
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset frames;

    rs2::pointcloud pointcloud;
    rs2::points points;
    rs2::frame colorFrame;
    rs2::frame depthFrame;
    rs2::frame irFrame;

    rs2::colorizer color_map;
    bool doAlignToColor{false};
    std::unique_ptr<rs2::align> alignToColor;
};

RealSense::RealSense()
    : m_pimpl(std::make_unique<Impl>())
{
}

RealSense::~RealSense()
{
    m_pimpl->stopStream();
}

bool RealSense::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr std::string_view logPrefix = "[RealSense::initialize] ";
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        return false;
    }

    m_pimpl->metadata.sensorsList.rgbCamerasList = std::vector<std::string>{"RealSense"};
    if (!ptr->getParameter("camera_name", m_pimpl->metadata.sensorsList.rgbCamerasList[0]))
    {
        log()->warn("{} Parameter \"camera_name\" not available in the configuration."
                    "Using default name \"{}\".",
                    logPrefix,
                    m_pimpl->metadata.sensorsList.rgbCamerasList[0]);
    }
    m_pimpl->metadata.sensorsList.rgbdCamerasList = m_pimpl->metadata.sensorsList.rgbCamerasList;

    //
    const auto& camName = m_pimpl->metadata.sensorsList.rgbCamerasList[0];
    m_pimpl->metadata.bridgeOptions.rgbImgDimensions[camName]
        = std::pair<std::size_t, std::size_t>{640, 480};
    auto& rgbDimensions = m_pimpl->metadata.bridgeOptions.rgbImgDimensions[camName];
    int width;
    if (ptr->getParameter("frame_width", width))
    {
        rgbDimensions.first = width;
    } else
    {
        log()->warn("{} Parameter \"frame_width\" not available in the configuration."
                    "Using default name \"{}\".",
                    logPrefix,
                    rgbDimensions.first);
    }
    int height;
    if (!ptr->getParameter("frame_height", height))
    {
        rgbDimensions.second = height;
    } else
    {
        log()->warn("{} Parameter \"frame_height\" not available in the configuration."
                    "Using default name \"{}\".",
                    logPrefix,
                    rgbDimensions.second);
    }
    m_pimpl->metadata.bridgeOptions.rgbdImgDimensions
        = m_pimpl->metadata.bridgeOptions.rgbImgDimensions;
    m_pimpl->metadata.bridgeOptions.depthImgDimensions
        = m_pimpl->metadata.bridgeOptions.rgbImgDimensions;

    if (!ptr->getParameter("fps", m_pimpl->genericFPS))
    {
        log()->warn("{} Parameter \"fps\" not available in the configuration."
                    "Using default name \"{}\".",
                    logPrefix,
                    m_pimpl->genericFPS);
    }

    m_pimpl->metadata.bridgeOptions.isRGBCameraEnabled = true;
    if (!ptr->getParameter("stream_color", m_pimpl->metadata.bridgeOptions.isRGBCameraEnabled))
    {
        log()->warn("{} Parameter \"stream_color\" not available in the configuration."
                    "Color stream will not be available.",
                    logPrefix);
    }

    m_pimpl->metadata.bridgeOptions.isRGBDCameraEnabled = false;
    if (!ptr->getParameter("stream_depth", m_pimpl->metadata.bridgeOptions.isRGBDCameraEnabled))
    {
        log()->warn("{} Parameter \"stream_depth\" not available in the configuration."
                    "Depth stream will not be available.",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_ir", m_pimpl->isIREnabled))
    {
        log()->warn("{} Parameter \"stream_ir\" not available in the configuration."
                    "IR stream will not be available.",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_pcl", m_pimpl->isPCLEnabled))
    {
        log()->warn("{} Parameter \"stream_pcl\" not available in the configuration."
                    "PCL stream will not be available.",
                    logPrefix);
    }

    if (!ptr->getParameter("align_frames_to_color", m_pimpl->doAlignToColor))
    {
        log()->warn("{} Parameter \"align_frames_to_color\" not available in the configuration."
                    "Frames will not be aligned to color frame.",
                    logPrefix);
    }

    if (!m_pimpl->startStream())
    {
        return false;
    }

    if (m_pimpl->doAlignToColor)
    {
        m_pimpl->alignToColor = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
    }
    return true;
}

bool RealSense::isValid() const
{
    if (!m_pimpl->isStreaming)
    {
        log()->error("[RealSenseCapture::isValid] Stream not active.");
        return false;
    }

    return true;
}

bool RealSense::Impl::isValidCamera(const std::string& _camName)
{
    if (this->metadata.sensorsList.rgbCamerasList.size() == 0
        || _camName != this->metadata.sensorsList.rgbCamerasList[0])
    {
        log()->error("[RealSenseCapture::Impl::isValidCamera] Requested camera not available.");
        return false;
    }

    return true;
}

bool RealSense::getRGBCamerasList(std::vector<std::string>& rgbCamerasList)
{
    rgbCamerasList = m_pimpl->metadata.sensorsList.rgbCamerasList;
    return true;
}

bool RealSense::getRGBDCamerasList(std::vector<std::string>& rgbdCamerasList)
{
    rgbdCamerasList = m_pimpl->metadata.sensorsList.rgbdCamerasList;
    return true;
}

bool RealSense::getPCLDevicesList(std::vector<std::string>& pclDevList)
{
    pclDevList = m_pimpl->metadata.sensorsList.rgbCamerasList;
    return true;
}

bool RealSense::getColorImage(const std::string& camName,
                              cv::Mat& colorImg,
                              std::optional<std::reference_wrapper<double>>)
{
    if (!isValid() || !m_pimpl->isValidCamera(camName))
    {
        return false;
    }

    if (!m_pimpl->metadata.bridgeOptions.isRGBCameraEnabled)
    {
        log()->error("[RealSenseCapture::getColorImage] Color stream was not enabled.");
        return false;
    }

    const std::size_t genericWidth
        = m_pimpl->metadata.bridgeOptions
              .rgbImgDimensions[m_pimpl->metadata.sensorsList.rgbCamerasList[0]]
              .first;
    const std::size_t genericHeight
        = m_pimpl->metadata.bridgeOptions
              .rgbImgDimensions[m_pimpl->metadata.sensorsList.rgbCamerasList[0]]
              .second;

    m_pimpl->frames = m_pimpl->pipe.wait_for_frames();
    if (m_pimpl->doAlignToColor && m_pimpl->alignToColor != nullptr)
    {
        m_pimpl->frames = m_pimpl->alignToColor->process(m_pimpl->frames);
    }
    m_pimpl->colorFrame = m_pimpl->frames.get_color_frame();
    colorImg = cv::Mat(cv::Size(genericWidth, genericHeight),
                       CV_8UC3,
                       (void*)m_pimpl->colorFrame.get_data(),
                       cv::Mat::AUTO_STEP);

    return true;
}

bool RealSense::getDepthImage(const std::string& camName,
                              cv::Mat& depthImg,
                              std::optional<std::reference_wrapper<double>>)
{
    if (!isValid() || !m_pimpl->isValidCamera(camName))
    {
        return false;
    }

    if (!m_pimpl->metadata.bridgeOptions.isRGBDCameraEnabled)
    {
        log()->error("[RealSenseCapture::getDepthImage] Depth stream was not enabled.");
        return false;
    }

    const std::size_t genericWidth
        = m_pimpl->metadata.bridgeOptions
              .depthImgDimensions[m_pimpl->metadata.sensorsList.rgbdCamerasList[0]]
              .first;
    const std::size_t genericHeight
        = m_pimpl->metadata.bridgeOptions
              .depthImgDimensions[m_pimpl->metadata.sensorsList.rgbdCamerasList[0]]
              .second;

    m_pimpl->frames = m_pimpl->pipe.wait_for_frames();
    if (m_pimpl->doAlignToColor && m_pimpl->alignToColor != nullptr)
    {
        m_pimpl->frames = m_pimpl->alignToColor->process(m_pimpl->frames);
    }
    m_pimpl->depthFrame = m_pimpl->frames.get_depth_frame();

    depthImg = cv::Mat(cv::Size(genericWidth, genericHeight),
                       CV_8UC1,
                       (void*)m_pimpl->depthFrame.get_data(),
                       cv::Mat::AUTO_STEP);

    return true;
}

bool RealSense::getColorizedDepthImage(const std::string& camName,
                                       cv::Mat& depthImg,
                                       std::optional<std::reference_wrapper<double>>)
{
    if (!isValid() || !m_pimpl->isValidCamera(camName))
    {
        return false;
    }

    if (!m_pimpl->metadata.bridgeOptions.isRGBDCameraEnabled)
    {
        log()->error("[RealSenseCapture::getColorizedDepthImage] Depth stream was not enabled.");
        return false;
    }

    const std::size_t genericWidth
        = m_pimpl->metadata.bridgeOptions
              .depthImgDimensions[m_pimpl->metadata.sensorsList.rgbdCamerasList[0]]
              .first;
    const std::size_t genericHeight
        = m_pimpl->metadata.bridgeOptions
              .depthImgDimensions[m_pimpl->metadata.sensorsList.rgbdCamerasList[0]]
              .second;

    m_pimpl->frames = m_pimpl->pipe.wait_for_frames();
    if (m_pimpl->doAlignToColor && m_pimpl->alignToColor != nullptr)
    {
        m_pimpl->frames = m_pimpl->alignToColor->process(m_pimpl->frames);
    }
    m_pimpl->depthFrame = m_pimpl->frames.get_depth_frame().apply_filter(m_pimpl->color_map);

    depthImg = cv::Mat(cv::Size(genericWidth, genericHeight),
                       CV_8UC3,
                       (void*)m_pimpl->depthFrame.get_data(),
                       cv::Mat::AUTO_STEP);

    return true;
}

bool RealSense::getInfraredImage(const std::string& camName,
                                 cv::Mat& irImage,
                                 std::optional<std::reference_wrapper<double>>)
{
    if (!isValid() || !m_pimpl->isValidCamera(camName))
    {
        return false;
    }

    if (!m_pimpl->isIREnabled)
    {
        log()->error("[RealSenseCapture::getInfraredImage] IR stream was not enabled.");
        return false;
    }

    const std::size_t genericWidth
        = m_pimpl->metadata.bridgeOptions.rgbImgDimensions[m_pimpl->metadata.sensorsList.rgbCamerasList[0]]
              .first;
    const std::size_t genericHeight
        = m_pimpl->metadata.bridgeOptions.rgbImgDimensions[m_pimpl->metadata.sensorsList.rgbCamerasList[0]]
              .second;

    m_pimpl->frames = m_pimpl->pipe.wait_for_frames();
    if (m_pimpl->doAlignToColor && m_pimpl->alignToColor != nullptr)
    {
        m_pimpl->frames = m_pimpl->alignToColor->process(m_pimpl->frames);
    }
    m_pimpl->irFrame = m_pimpl->frames.get_infrared_frame();
    irImage = cv::Mat(cv::Size(genericWidth, genericHeight),
                      CV_8UC1,
                      (void*)m_pimpl->irFrame.get_data(),
                      cv::Mat::AUTO_STEP);

    return true;
}

bool RealSense::getPointCloud(const std::string& pclDevName,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPointCloud,
                              std::optional<std::reference_wrapper<double>>)
{
    if (!isValid() || !m_pimpl->isValidCamera(pclDevName))
    {
        return false;
    }

    if (!m_pimpl->isPCLEnabled)
    {
        log()->error("[RealSenseCapture::getPointCloud] PCL stream was not enabled.");
        return false;
    }

    if (coloredPointCloud == nullptr)
    {
        log()->error("[RealSenseCapture::getPointCloud] Did not receive a valid pointer.");
        return false;
    }

    // Wait for the next set of frames from the camera
    m_pimpl->frames = m_pimpl->pipe.wait_for_frames();
    if (m_pimpl->doAlignToColor && m_pimpl->alignToColor != nullptr)
    {
        m_pimpl->frames = m_pimpl->alignToColor->process(m_pimpl->frames);
    }
    m_pimpl->depthFrame = m_pimpl->frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    m_pimpl->points = m_pimpl->pointcloud.calculate(m_pimpl->depthFrame);

    m_pimpl->colorFrame = m_pimpl->frames.get_color_frame();

    // Tell pointcloud object to map to this color frame
    m_pimpl->pointcloud.map_to(m_pimpl->colorFrame);

    m_pimpl->toPCL(m_pimpl->points, m_pimpl->colorFrame, coloredPointCloud);
    return true;
}

bool RealSense::Impl::startStream()
{
    if (isPCLEnabled)
    {
        metadata.bridgeOptions.isRGBCameraEnabled = true;
        metadata.bridgeOptions.isRGBDCameraEnabled = true;

        log()->info("[RealSenseCapture::Impl::startStream] PCL stream enabled. Automatically "
                    "enabling RGB and Depth streaming.");
    }

    if (!metadata.bridgeOptions.isRGBCameraEnabled && !isIREnabled
        && !metadata.bridgeOptions.isRGBDCameraEnabled)
    {
        log()->error("[RealSenseCapture::Impl::startStream] None of the stream options enabled. "
                     "Cannot start streaming.");
        return false;
    }

    // by construction the width and height are the same for the rgbd and the rgb camera
    const std::size_t genericWidth
        = metadata.bridgeOptions.rgbImgDimensions[metadata.sensorsList.rgbCamerasList[0]].first;
    const std::size_t genericHeight
        = metadata.bridgeOptions.rgbImgDimensions[metadata.sensorsList.rgbCamerasList[0]].second;

    if (metadata.bridgeOptions.isRGBCameraEnabled)
    {
        cfg.enable_stream(RS2_STREAM_COLOR, genericWidth, genericHeight, colorFormat, genericFPS);
    }

    if (isIREnabled)
    {
        cfg.enable_stream(RS2_STREAM_INFRARED, genericWidth, genericHeight, irFormat, genericFPS);
    }

    if (metadata.bridgeOptions.isRGBDCameraEnabled)
    {
        cfg.enable_stream(RS2_STREAM_DEPTH, genericWidth, genericHeight, depthFormat, genericFPS);
    }

    log()->info("[RealSenseCapture::Impl::startStream] Trying to connect to device.");

    try
    {
        pipe.start(cfg);
    } catch (const rs2::error& e)
    {
        log()->error("[RealSenseCapture::Impl::startStream] Failed to start the pipeline: ({})",
                     e.what());
        return false;
    }

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    for (int i = 0; i < 30; i++)
    {
        // Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }

    isStreaming = true;
    return true;
}

void RealSense::Impl::stopStream()
{
    pipe.stop();
    cfg.disable_all_streams();
    isStreaming = false;
}

//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values.
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
RealSense::Impl::TextureRGB
RealSense::Impl::rgbTexture(rs2::video_frame textureImg, rs2::texture_coordinate textureXY)
{
    // Get Width and Height coordinates of texture
    int width = textureImg.get_width(); // Frame width in pixels
    int height = textureImg.get_height(); // Frame height in pixels

    // Normals to Texture Coordinates conversion
    int xValue = std::min(std::max(int(textureXY.u * width + .5f), 0), width - 1);
    int yValue = std::min(std::max(int(textureXY.v * height + .5f), 0), height - 1);

    int bytes = xValue * textureImg.get_bytes_per_pixel(); // Get # of bytes per pixel
    int strides = yValue * textureImg.get_stride_in_bytes(); // Get line width in bytes
    int textureIndex = (bytes + strides);

    const auto newTexture = reinterpret_cast<const uint8_t*>(textureImg.get_data());

    // RGB components to save in tuple
    int NT1 = newTexture[textureIndex];
    int NT2 = newTexture[textureIndex + 1];
    int NT3 = newTexture[textureIndex + 2];

    return RealSense::Impl::TextureRGB{NT1, NT2, NT3};
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//===================================================
void RealSense::Impl::toPCL(const rs2::points& points,
                            const rs2::video_frame& color,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Declare struct for RGB value Storage (<r>, <g>, <b>)
    RealSense::Impl::TextureRGB rgbColor;
    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    if (cloud == nullptr)
    {
        return;
    }

    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto textureCoord = points.get_texture_coordinates();
    auto vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = vertex[i].x;
        cloud->points[i].y = vertex[i].y;
        cloud->points[i].z = vertex[i].z;

        // Obtain color texture for specific point
        rgbColor = rgbTexture(color, textureCoord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = rgbColor.b; // Reference tuple<2>
        cloud->points[i].g = rgbColor.g; // Reference tuple<1>
        cloud->points[i].b = rgbColor.r; // Reference tuple<0>
    }
}

const BipedalLocomotion::RobotInterface::CameraBridgeMetaData& RealSense::getMetaData() const
{
    return m_pimpl->metadata;
}
