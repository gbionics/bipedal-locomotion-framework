/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_IMAGE_WRITERS_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_IMAGE_WRITERS_H

#include <filesystem>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace BipedalLocomotion
{
namespace RobotLogger
{

enum class ImageSaveMode
{
    Video, /**< The images are stored in a video file. */
    Frames /**< Each image is stored in a png file. */
};

struct VideoEncoderOptions
{
    /** FFmpeg encoder of the color videos (e.g., libx264). If empty the first available among
     * libx264, libopenh264 and mpeg4 is used. */
    std::string encoder;

    /** FOURCC code used by the OpenCV writer, i.e., when FFmpeg is not available. */
    std::string fourcc{"mp4v"};

    /** Nominal frame rate. */
    double fps{30.0};
};

/**
 * IImageWriter stores a sequence of images and their timestamps. The images are either 8-bit BGR
 * (CV_8UC3) or 16-bit single channel (CV_16UC1) images, e.g., depth in millimeters.
 *
 * The video writers store each image at time (time - firstImageTime), i.e., the position of an
 * image in the video corresponds to its timestamp. This guarantees the alignment with the other
 * logged signals also if some images are dropped.
 */
class IImageWriter
{
public:
    virtual ~IImageWriter() = default;

    /** Extension of the file (e.g., ".mp4"). It is empty if the images are saved in a folder. */
    virtual std::string extension() const = 0;

    virtual bool open(const std::filesystem::path& path,
                      const cv::Mat& firstImage,
                      double firstImageTime)
        = 0;

    /**
     * Write an image.
     * @return false if the image has not been written.
     */
    virtual bool write(const cv::Mat& image, double time) = 0;

    virtual void close() = 0;
};

/**
 * Create a writer.
 * @param mode save mode.
 * @param isDepth true if the images are 16-bit depth images.
 * @param options options of the video encoder.
 */
std::unique_ptr<IImageWriter>
createImageWriter(ImageSaveMode mode, bool isDepth, const VideoEncoderOptions& options);

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_IMAGE_WRITERS_H
