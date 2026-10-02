/**
 * @file ImageRecorder.h
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_IMAGE_RECORDER_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_IMAGE_RECORDER_H

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/RobotLogger/TelemetryBuffer.h>

namespace BipedalLocomotion
{
namespace RobotLogger
{

/**
 * IImageSource provides the images recorded by an ImageRecorder.
 */
class IImageSource
{
public:
    virtual ~IImageSource() = default;

    /**
     * Read an image. A source may block until a new image is available.
     * @param image the image. It must own its data.
     * @return true in case of success, false otherwise.
     */
    virtual bool read(cv::Mat& image) = 0;

    /**
     * Unblock a pending read.
     */
    virtual void interrupt()
    {
    }

    /**
     * Allow the reads again after interrupt().
     */
    virtual void resume()
    {
    }
};

/**
 * ImageRecorder saves the images provided by an IImageSource in a video or in a folder of png
 * files.
 *
 * The images are read by an acquisition thread and written by a writer thread, so that a slow
 * encoding does not affect the acquisition. If the writer cannot keep up, the new images are
 * dropped.
 *
 * The images are written in a temporary file in the log folder. When rotate() is called the file
 * is closed and renamed as `<fileName>_<name>_<image_type><extension>`, and the next images are
 * written in a new file. For each saved image, its index in the file and its time are stored in
 * the channel `channel` of the TelemetryBuffer. The index restarts from zero in every file.
 *
 * The videos are written with FFmpeg and each image is stored with its own timestamp: the rgb
 * videos are encoded in H.264 (fragmented mp4) and the depth videos with the lossless FFV1 codec
 * (mkv).
 */
class ImageRecorder
{
public:
    ImageRecorder();

    ~ImageRecorder();

    // clang-format off
    /**
     * Initialize the recorder.
     * @param handler pointer to the parameters handler.
     * @param source source of the images.
     * @param buffer buffer storing the index and the time of each saved image.
     * @note The following parameters are used:
     * |   Parameter Name   |   Type   |                       Description                        | Mandatory |
     * |:------------------:|:--------:|:--------------------------------------------------------:|:---------:|
     * |       `name`       | `string` |     Name used in the file names, e.g., the camera name.  |    Yes    |
     * |    `image_type`    | `string` | `rgb` (CV_8UC3 images) or `depth` (CV_16UC1 images).     |    Yes    |
     * |     `channel`      | `string` |    Channel storing the index and the time of the images. |    Yes    |
     * |    `save_mode`     | `string` |                 Either `video` or `frame`.               |    Yes    |
     * |       `fps`        | `double` | Acquisition rate. If 0 the images are read as soon as they arrive. Default 0. | No |
     * |  `video_encoder`   | `string` | FFmpeg encoder of the rgb videos. Default the first available among `libx264`, `libopenh264` and `mpeg4`. | No |
     * @return true in case of success, false otherwise.
     */
    // clang-format on
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                    std::unique_ptr<IImageSource> source,
                    std::shared_ptr<TelemetryBuffer> buffer);

    /**
     * Add the channel to the buffer and start the acquisition.
     */
    bool start();

    /**
     * Stop the acquisition and wait until all the acquired images are written.
     */
    void stopAcquisition();

    /**
     * Close the current file and rename it with the given file name (without extension).
     */
    void rotate(const std::string& fileName);

    /**
     * Close and delete the current file.
     */
    void discard();

    /**
     * Stop the acquisition and the writer.
     */
    void stop();

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_IMAGE_RECORDER_H
