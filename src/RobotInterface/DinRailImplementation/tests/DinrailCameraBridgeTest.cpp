/**
 * @file DinrailCameraBridgeTest.cpp
 * @copyright Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <memory>
#include <thread>

#include <dinrail/IRGBDSensor.h>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/RobotInterface/DinrailCameraBridge.h>
#include <BipedalLocomotion/RobotInterface/DinrailHelper.h>

using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::ParametersHandler;

namespace
{

constexpr int width = 32;
constexpr int height = 24;

DinrailDeviceDescriptor openFakeCamera(bool enableRgb)
{
    dinrail::Parameters parameters;
    parameters.put("device", "dr_rgbd_fake");
    parameters.put("width", width);
    parameters.put("height", height);
    parameters.put("depth", 2.5);
    parameters.put("enable_rgb", enableRgb);
    DinrailDeviceDescriptor camera("camera", std::make_shared<dinrail::Device>());
    REQUIRE(camera.device->open(parameters));
    return camera;
}

std::shared_ptr<StdImplementation> bridgeParameters()
{
    auto cameras = std::make_shared<StdImplementation>();
    cameras->setParameter("rgbd_cameras_list", std::vector<std::string>{"camera"});
    auto parameters = std::make_shared<StdImplementation>();
    parameters->setParameter("stream_cameras", true);
    parameters->setGroup("Cameras", cameras);
    return parameters;
}

} // namespace

TEST_CASE("DinrailCameraBridge reads the images of a dinrail RGBD sensor")
{
    const DinrailDeviceDescriptor camera = openFakeCamera(true);
    DinrailCameraBridge bridge;
    REQUIRE(bridge.initialize(bridgeParameters()));
    REQUIRE(bridge.setDevicesList({camera}));
    REQUIRE(bridge.isValid());

    std::vector<std::string> cameras;
    REQUIRE(bridge.getRGBDCamerasList(cameras));
    REQUIRE(cameras == std::vector<std::string>{"camera"});
    // The sizes are retrieved from the intrinsic parameters of the camera.
    const auto& options = bridge.getMetaData().bridgeOptions;
    REQUIRE(options.rgbdImgDimensions.at("camera")
            == std::pair<std::size_t, std::size_t>{width, height});
    REQUIRE(options.depthImgDimensions.at("camera")
            == std::pair<std::size_t, std::size_t>{width, height});

    cv::Mat depth;
    std::chrono::nanoseconds depthTime{0};
    REQUIRE(bridge.getDepthImage("camera", depth, depthTime));
    REQUIRE(depth.type() == CV_32FC1);
    REQUIRE(depth.cols == width);
    REQUIRE(depth.rows == height);
    REQUIRE(depth.at<float>(3, 5) == 2.5F);

    SECTION("The receive time changes only for a new frame")
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        std::chrono::nanoseconds sameFrameTime{0};
        REQUIRE(bridge.getDepthImage("camera", depth, sameFrameTime));
        REQUIRE(sameFrameTime == depthTime);

        dinrail::IRGBDSensorSimulation* simulation = nullptr;
        REQUIRE(camera.device->view(simulation));
        dinrail::Image rgb;
        dinrail::DepthImage newDepth;
        dinrail::IRGBDSensor* sensor = nullptr;
        REQUIRE(camera.device->view(sensor));
        REQUIRE(sensor->getImages(rgb, newDepth).ok());
        newDepth.data.assign(newDepth.data.size(), 1.0F);
        newDepth.stamp.sequenceNumber = 1;
        REQUIRE(simulation->setImages(rgb, newDepth).ok());

        std::chrono::nanoseconds newFrameTime{0};
        REQUIRE(bridge.getDepthImage("camera", depth, newFrameTime));
        REQUIRE(newFrameTime > depthTime);
        REQUIRE(depth.at<float>(3, 5) == 1.0F);
    }

    SECTION("The color image is converted to BGR")
    {
        dinrail::IRGBDSensor* sensor = nullptr;
        dinrail::IRGBDSensorSimulation* simulation = nullptr;
        REQUIRE(camera.device->view(sensor));
        REQUIRE(camera.device->view(simulation));
        dinrail::Image rgb;
        dinrail::DepthImage depthImage;
        REQUIRE(sensor->getImages(rgb, depthImage).ok());
        for (std::size_t i = 0; i < rgb.data.size(); i += 3)
        {
            rgb.data[i] = 10;
            rgb.data[i + 1] = 20;
            rgb.data[i + 2] = 30;
        }
        REQUIRE(simulation->setImages(rgb, depthImage).ok());

        cv::Mat color;
        REQUIRE(bridge.getColorImage("camera", color));
        REQUIRE(color.type() == CV_8UC3);
        REQUIRE(color.cols == width);
        REQUIRE(color.rows == height);
        REQUIRE(color.at<cv::Vec3b>(3, 5) == cv::Vec3b(30, 20, 10));
    }

    SECTION("Unknown cameras are reported")
    {
        REQUIRE_FALSE(bridge.getDepthImage("unknown", depth));
    }
}

TEST_CASE("DinrailCameraBridge reads the depth of a camera without color stream")
{
    const DinrailDeviceDescriptor camera = openFakeCamera(false);
    DinrailCameraBridge bridge;
    REQUIRE(bridge.initialize(bridgeParameters()));
    REQUIRE(bridge.setDevicesList({camera}));
    REQUIRE(bridge.getMetaData().bridgeOptions.rgbdImgDimensions.at("camera")
            == std::pair<std::size_t, std::size_t>{0, 0});

    cv::Mat depth;
    REQUIRE(bridge.getDepthImage("camera", depth));
    REQUIRE(depth.at<float>(0, 0) == 2.5F);
    cv::Mat color;
    REQUIRE_FALSE(bridge.getColorImage("camera", color));
}

TEST_CASE("DinrailCameraBridge configuration errors")
{
    DinrailCameraBridge bridge;

    SECTION("The devices must be set after initialize")
    {
        REQUIRE_FALSE(bridge.setDevicesList({}));
        REQUIRE_FALSE(bridge.isValid());
    }

    SECTION("All the configured cameras must be provided")
    {
        REQUIRE(bridge.initialize(bridgeParameters()));
        REQUIRE_FALSE(bridge.setDevicesList({}));
        REQUIRE_FALSE(bridge.isValid());
    }

    SECTION("RGB-only cameras are not supported")
    {
        auto parameters = bridgeParameters();
        parameters->getGroup("Cameras").lock()->setParameter("rgb_cameras_list",
                                                             std::vector<std::string>{"webcam"});
        REQUIRE_FALSE(bridge.initialize(parameters));
    }

    SECTION("The image sizes must match the cameras")
    {
        auto parameters = bridgeParameters();
        auto cameras = parameters->getGroup("Cameras").lock();
        cameras->setParameter("rgbd_depth_image_width", std::vector<int>{width, width});
        cameras->setParameter("rgbd_depth_image_height", std::vector<int>{height, height});
        REQUIRE_FALSE(bridge.initialize(parameters));
    }
}

TEST_CASE("constructDinrailRGBDSensorClient requires the ports")
{
    auto parameters = std::make_shared<StdImplementation>();
    parameters->setParameter("name", "camera");
    parameters->setParameter("local_prefix", "test");
    parameters->setParameter("local_depth_port_postfix", "/depth:i");
    REQUIRE_FALSE(constructDinrailRGBDSensorClient(parameters).isValid());
}
