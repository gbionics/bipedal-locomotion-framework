/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_FRAME_TRANSFORM_LOGGER_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_FRAME_TRANSFORM_LOGGER_H

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Matrix.h>

#include <BipedalLocomotion/RobotLogger/DataSink.h>

namespace BipedalLocomotion
{
namespace RobotLogger
{

/**
 * FrameTransformLogger logs the transforms published on the yarp transform server that can be
 * expressed with respect to a set of parent frames.
 */
class FrameTransformLogger
{
public:
    /**
     * @param config the `Transforms` group. It contains the list `parent_frames` and the group
     * `TransformClientDevice` used to open the transform client.
     */
    bool initialize(const yarp::os::Bottle& config);

    /** Forget the known frames. It must be called at the beginning of each recording session. */
    void reset();

    void record(DataSink& sink, double time);

private:
    struct FrameDescriptor
    {
        std::string parent;
        std::string positionChannel;
        std::string orientationChannel;
        bool active{true};
    };

    void updateFrames(DataSink& sink);

    yarp::dev::PolyDriver m_device;
    yarp::dev::IFrameTransform* m_transform{nullptr};
    std::unordered_set<std::string> m_parentFrames;
    std::unordered_map<std::string, FrameDescriptor> m_frames;
    std::vector<std::string> m_allFrames;
    yarp::sig::Matrix m_matrix;
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_FRAME_TRANSFORM_LOGGER_H
