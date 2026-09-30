/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_FFMPEG_VIDEO_WRITER_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_FFMPEG_VIDEO_WRITER_H

#include <cstdint>
#include <string>

#include <BipedalLocomotion/RobotLogger/ImageWriters.h>

struct AVCodecContext;
struct AVFormatContext;
struct AVFrame;
struct AVPacket;
struct AVStream;
struct SwsContext;

namespace BipedalLocomotion
{
namespace RobotLogger
{

/**
 * Video writer based on FFmpeg. The presentation timestamp of each image is its logging time, so
 * the video has a variable frame rate and it is aligned with the other logged signals.
 * - Color images are encoded in H.264 (or MPEG-4) and stored in fragmented mp4 files, which can
 *   be read also if the process crashes while writing them.
 * - Depth images are encoded with the lossless FFV1 codec (16 bit) and stored in mkv files.
 * The time of the first image is stored in the `blf_first_frame_time` metadata of the file.
 */
class FFmpegVideoWriter final : public IImageWriter
{
public:
    FFmpegVideoWriter(bool isDepth, VideoEncoderOptions options);

    ~FFmpegVideoWriter() override;

    std::string extension() const final;

    bool open(const std::filesystem::path& path,
              const cv::Mat& firstImage,
              double firstImageTime) final;

    bool write(const cv::Mat& image, double time) final;

    void close() final;

private:
    bool openEncoder(const std::string& name);
    bool encode(AVFrame* frame);

    bool m_isDepth;
    VideoEncoderOptions m_options;

    AVFormatContext* m_format{nullptr};
    AVCodecContext* m_codec{nullptr};
    AVStream* m_stream{nullptr};
    AVFrame* m_frame{nullptr};
    AVPacket* m_packet{nullptr};
    SwsContext* m_scaler{nullptr};
    bool m_headerWritten{false};

    int m_inputWidth{0};
    int m_inputHeight{0};
    int m_width{0};
    int m_height{0};
    double m_firstTime{0};
    std::int64_t m_lastPts{-1};
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_FFMPEG_VIDEO_WRITER_H
