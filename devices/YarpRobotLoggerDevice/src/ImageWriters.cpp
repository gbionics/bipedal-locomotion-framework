/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <cmath>
#include <system_error>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotLogger/ImageWriters.h>

#ifdef BLF_YARP_ROBOT_LOGGER_USE_FFMPEG
#include <BipedalLocomotion/RobotLogger/FFmpegVideoWriter.h>
#endif

using namespace BipedalLocomotion::RobotLogger;

namespace
{

/** Store each image in a png file named img_<index>.png. */
class FramesWriter final : public IImageWriter
{
public:
    std::string extension() const final
    {
        return "";
    }

    bool open(const std::filesystem::path& path, const cv::Mat&, double) final
    {
        std::error_code ec;
        std::filesystem::create_directories(path, ec);
        if (ec)
        {
            BipedalLocomotion::log()->error("[FramesWriter::open] Unable to create the folder {}. "
                                            "Error: {}.",
                                            path.string(),
                                            ec.message());
            return false;
        }
        m_folder = path;
        m_index = 0;
        return true;
    }

    bool write(const cv::Mat& image, double) final
    {
        const auto path = m_folder / ("img_" + std::to_string(m_index) + ".png");
        if (!cv::imwrite(path.string(), image))
        {
            return false;
        }
        m_index++;
        return true;
    }

    void close() final
    {
    }

private:
    std::filesystem::path m_folder;
    std::size_t m_index{0};
};

/**
 * Video writer based on cv::VideoWriter. The video has a constant frame rate, so the last image is
 * repeated to fill the gaps and the images arriving faster than the frame rate are skipped.
 */
class OpenCVVideoWriter final : public IImageWriter
{
public:
    OpenCVVideoWriter(bool isDepth, VideoEncoderOptions options)
        : m_isDepth(isDepth)
        , m_options(std::move(options))
    {
    }

    std::string extension() const final
    {
        return ".mp4";
    }

    bool open(const std::filesystem::path& path, const cv::Mat& firstImage, double firstTime) final
    {
        if (m_options.fourcc.size() != 4)
        {
            BipedalLocomotion::log()->error("[OpenCVVideoWriter::open] The fourcc code must "
                                            "contain 4 characters. Provided: {}.",
                                            m_options.fourcc);
            return false;
        }

        const auto& code = m_options.fourcc;
        m_writer.open(path.string(),
                      cv::VideoWriter::fourcc(code[0], code[1], code[2], code[3]),
                      m_options.fps,
                      firstImage.size(),
                      !m_isDepth);
        m_firstTime = firstTime;
        m_writtenFrames = 0;
        m_lastImage.release();
        return m_writer.isOpened();
    }

    bool write(const cv::Mat& image, double time) final
    {
        const long long slot = std::llround((time - m_firstTime) * m_options.fps);
        if (slot < m_writtenFrames)
        {
            return false;
        }

        cv::Mat frame = image;
        if (m_isDepth)
        {
            image.convertTo(frame, CV_8UC1);
        }

        while (m_writtenFrames < slot && !m_lastImage.empty())
        {
            m_writer.write(m_lastImage);
            m_writtenFrames++;
        }

        m_writer.write(frame);
        m_lastImage = frame;
        m_writtenFrames = slot + 1;
        return true;
    }

    void close() final
    {
        m_writer.release();
    }

private:
    bool m_isDepth;
    VideoEncoderOptions m_options;
    cv::VideoWriter m_writer;
    cv::Mat m_lastImage;
    double m_firstTime{0};
    long long m_writtenFrames{0};
};

} // namespace

std::unique_ptr<IImageWriter> BipedalLocomotion::RobotLogger::createImageWriter(
    ImageSaveMode mode, bool isDepth, const VideoEncoderOptions& options)
{
    if (mode == ImageSaveMode::Frames)
    {
        return std::make_unique<FramesWriter>();
    }

#ifdef BLF_YARP_ROBOT_LOGGER_USE_FFMPEG
    return std::make_unique<FFmpegVideoWriter>(isDepth, options);
#else
    return std::make_unique<OpenCVVideoWriter>(isDepth, options);
#endif
}
