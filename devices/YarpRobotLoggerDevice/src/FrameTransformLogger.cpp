/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <memory>

#include <Eigen/Geometry>

#include <yarp/conf/version.h>
#include <yarp/eigen/Eigen.h>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotLogger/FrameTransformLogger.h>

using namespace BipedalLocomotion::RobotLogger;

bool FrameTransformLogger::initialize(const yarp::os::Bottle& config)
{
    constexpr auto logPrefix = "[FrameTransformLogger::initialize]";

    if (config.isNull())
    {
        log()->error("{} The 'Transforms' group is not provided.", logPrefix);
        return false;
    }

    auto params = std::make_shared<ParametersHandler::YarpImplementation>(config);
    std::vector<std::string> parents;
    if (!params->getParameter("parent_frames", parents) || parents.empty())
    {
        log()->error("{} The 'parent_frames' parameter is missing or empty.", logPrefix);
        return false;
    }
    m_parentFrames.insert(parents.begin(), parents.end());

    // the group is passed as it is to the device
    yarp::os::Bottle& deviceGroup = config.findGroup("TransformClientDevice");
    if (deviceGroup.isNull())
    {
        log()->error("{} The 'TransformClientDevice' group is not provided.", logPrefix);
        return false;
    }

    if (!m_device.open(deviceGroup))
    {
        log()->error("{} Unable to open the transform client.", logPrefix);
        return false;
    }

    if (!m_device.view(m_transform) || m_transform == nullptr)
    {
        log()->error("{} Unable to view the IFrameTransform interface.", logPrefix);
        return false;
    }

    return true;
}

void FrameTransformLogger::reset()
{
    m_frames.clear();
}

void FrameTransformLogger::updateFrames(DataSink& sink)
{
    for (auto& [name, frame] : m_frames)
    {
        frame.active = false;
    }

    // the vector is not cleared by getAllFrameIds
    m_allFrames.clear();
    if (!m_transform->getAllFrameIds(m_allFrames))
    {
        return;
    }

    for (const auto& id : m_allFrames)
    {
        if (m_parentFrames.find(id) != m_parentFrames.end())
        {
            continue;
        }

        const auto known = m_frames.find(id);
        if (known != m_frames.end())
        {
            known->second.active = true;
            continue;
        }

        for (const auto& parent : m_parentFrames)
        {
#if YARP_VERSION_COMPARE(<, 3, 11, 0)
            const bool canTransform = m_transform->canTransform(id, parent);
#else
            bool ok = false;
            const bool canTransform = m_transform->canTransform(id, parent, ok) && ok;
#endif
            if (!canTransform)
            {
                continue;
            }

            FrameDescriptor frame;
            frame.parent = parent;
            frame.positionChannel = "frames::" + parent + "::" + id + "::position";
            frame.orientationChannel = "frames::" + parent + "::" + id + "::orientation";

            // if the channels cannot be added now the frame is added at the next call
            if (sink.addChannel(frame.positionChannel, 3, {"x", "y", "z"})
                && sink.addChannel(frame.orientationChannel, 4, {"qx", "qy", "qz", "qw"}))
            {
                m_frames.emplace(id, frame);
            }
            break;
        }
    }
}

void FrameTransformLogger::record(DataSink& sink, double time)
{
    this->updateFrames(sink);

    for (const auto& [id, frame] : m_frames)
    {
        if (!frame.active || !m_transform->getTransform(id, frame.parent, m_matrix))
        {
            continue;
        }

        const Eigen::Matrix4d transform = yarp::eigen::toEigen(m_matrix);
        const Eigen::Vector3d position = transform.topRightCorner<3, 1>();
        const Eigen::Quaterniond quaternion(Eigen::Matrix3d(transform.topLeftCorner<3, 3>()));
        Eigen::Vector4d orientation;
        orientation << quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w();

        sink.log(frame.positionChannel, position, time);
        sink.log(frame.orientationChannel, orientation, time);
    }
}
