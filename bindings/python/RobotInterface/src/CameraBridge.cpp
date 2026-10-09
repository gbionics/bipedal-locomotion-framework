/**
 * @file CameraBridge.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#define PYBIND_VERSION_AT_LEAST(x, y, z)                                \
    ((pybind11_VERSION_MAJOR > x)                                       \
     || ((pybind11_VERSION_MAJOR >= x) && (pybind11_VERSION_MINOR > y)) \
     || ((pybind11_VERSION_MAJOR >= x) && (pybind11_VERSION_MINOR >= y) \
         && (pybind11_VERSION_PATCH > z)))

// This compiles only if pybind11 is at least v2.7.0
// Indeed we need a feature in pybind that has been introduced by this commit
// https://github.com/pybind/pybind11/commit/74a767d42921001fc4569ecee3b8726383c42ad4
// https://github.com/pybind/pybind11/pull/2864
#if PYBIND_VERSION_AT_LEAST(2, 7, 0)
// NumPy/OpenCV compatibility
#include <cvnp/cvnp.h>
#endif // PYBIND_VERSION_AT_LEAST(2, 7, 0)

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/RobotInterface/ICameraBridge.h>
#include <BipedalLocomotion/RobotInterface/YarpCameraBridge.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>

#include <BipedalLocomotion/System/Source.h>

#include <BipedalLocomotion/bindings/RobotInterface/CameraBridge.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{

void CreateICameraBridge(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::RobotInterface;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<CameraBridgeOptions>(module, "CameraBridgeOptions")
        .def(py::init())
        .def_readwrite("is_rgb_camera_enabled", &CameraBridgeOptions::isRGBCameraEnabled)
        .def_readwrite("is_rgbd_camera_enabled", &CameraBridgeOptions::isRGBDCameraEnabled)
        .def_readwrite("rgb_img_dimensions", &CameraBridgeOptions::rgbImgDimensions)
        .def_readwrite("rgbd_img_dimension", &CameraBridgeOptions::rgbdImgDimensions)
        .def_readwrite("depth_img_dimensions", &CameraBridgeOptions::depthImgDimensions);

    py::class_<CameraLists>(module, "CameraLists")
        .def(py::init())
        .def_readwrite("rgb_cameras_list", &CameraLists::rgbCamerasList)
        .def_readwrite("rgbd_cameras_list", &CameraLists::rgbdCamerasList);

    py::class_<CameraBridgeMetaData>(module, "CameraBridgeMetaData")
        .def(py::init())
        .def_readwrite("sensors_list", &CameraBridgeMetaData::sensorsList)
        .def_readwrite("bridge_options", &CameraBridgeMetaData::bridgeOptions);

    py::class_<ICameraBridge> iCameraBridge(module, "ICameraBridge");
    iCameraBridge
        .def(
            "initialize",
            [](ICameraBridge& impl, std::shared_ptr<const IParametersHandler> handler) -> bool {
                return impl.initialize(handler);
            },
            py::arg("handler"))
        .def("get_meta_data", &ICameraBridge::getMetaData)
        .def("is_valid", &ICameraBridge::isValid);

    // Here we require a feature in pybind that has been introduced by this commit
    // https://github.com/pybind/pybind11/commit/74a767d42921001fc4569ecee3b8726383c42ad4
    // https://github.com/pybind/pybind11/pull/2864
#if PYBIND_VERSION_AT_LEAST(2, 7, 0)
    iCameraBridge
        .def(
            "get_color_image",
            [](ICameraBridge& impl, const std::string& camName) {
                std::pair<bool, cv::Mat> ret;
                ret.first = impl.getColorImage(camName, ret.second);
                return ret;
            },
            py::arg("cam_name"))
        .def(
            "get_depth_image",
            [](ICameraBridge& impl, const std::string& camName) {
                std::pair<bool, cv::Mat> ret;
                ret.first = impl.getDepthImage(camName, ret.second);
                return ret;
            },
            py::arg("cam_name"));
#endif // PYBIND_VERSION_AT_LEAST(2, 7, 0)
}

void CreateYarpCameraBridge(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::RobotInterface;
    using namespace BipedalLocomotion::System;
    py::class_<YarpCameraBridge, ICameraBridge>(module, "YarpCameraBridge")
        .def(py::init())
        .def(
            "set_drivers_list",
            [](YarpCameraBridge& impl, std::vector<PolyDriverDescriptor> polydrivers) -> bool {
                yarp::dev::PolyDriverList list;
                for (const auto& polydriver : polydrivers)
                {
                    list.push(polydriver.poly.get(), polydriver.key.c_str());
                }
                return impl.setDriversList(list);
            },
            py::arg("polydrivers"));
}

} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
