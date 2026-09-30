/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/dict.h>
#include <libavutil/frame.h>
#include <libswscale/swscale.h>
}

#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotLogger/FFmpegVideoWriter.h>

using namespace BipedalLocomotion::RobotLogger;

namespace
{
std::string errorString(int error)
{
    char buffer[AV_ERROR_MAX_STRING_SIZE] = {0};
    av_strerror(error, buffer, sizeof(buffer));
    return buffer;
}

// the timestamps are expressed in milliseconds
constexpr AVRational timeBase{1, 1000};
} // namespace

FFmpegVideoWriter::FFmpegVideoWriter(bool isDepth, VideoEncoderOptions options)
    : m_isDepth(isDepth)
    , m_options(std::move(options))
{
}

FFmpegVideoWriter::~FFmpegVideoWriter()
{
    this->close();
}

std::string FFmpegVideoWriter::extension() const
{
    return m_isDepth ? ".mkv" : ".mp4";
}

bool FFmpegVideoWriter::openEncoder(const std::string& name)
{
    const AVCodec* codec = avcodec_find_encoder_by_name(name.c_str());
    if (codec == nullptr)
    {
        return false;
    }

    AVCodecContext* context = avcodec_alloc_context3(codec);
    context->width = m_width;
    context->height = m_height;
    context->time_base = timeBase;
    context->framerate = av_d2q(m_options.fps, 1000);
    context->pix_fmt = m_isDepth ? AV_PIX_FMT_GRAY16LE : AV_PIX_FMT_YUV420P;
    // a keyframe (and a new mp4 fragment) every two seconds
    context->gop_size = std::max(1, static_cast<int>(std::lround(2 * m_options.fps)));
    // without B-frames the first image is presented at time zero
    context->max_b_frames = 0;
    if (m_format->oformat->flags & AVFMT_GLOBALHEADER)
    {
        context->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    AVDictionary* options = nullptr;
    if (name == "libx264")
    {
        av_dict_set(&options, "preset", "veryfast", 0);
        av_dict_set(&options, "crf", "23", 0);
    } else if (!m_isDepth)
    {
        constexpr double bitsPerPixel = 0.15;
        context->bit_rate = static_cast<std::int64_t>(bitsPerPixel * m_width * m_height //
                                                      * m_options.fps);
    }

    const int ret = avcodec_open2(context, codec, &options);
    av_dict_free(&options);
    if (ret < 0)
    {
        log()->warn("[FFmpegVideoWriter::openEncoder] Unable to open the encoder {}. Error: {}.",
                    name,
                    errorString(ret));
        avcodec_free_context(&context);
        return false;
    }

    m_codec = context;
    return true;
}

bool FFmpegVideoWriter::open(const std::filesystem::path& path,
                             const cv::Mat& firstImage,
                             double firstImageTime)
{
    constexpr auto logPrefix = "[FFmpegVideoWriter::open]";

    this->close();

    const int expectedType = m_isDepth ? CV_16UC1 : CV_8UC3;
    if (firstImage.type() != expectedType || firstImage.empty())
    {
        log()->error("{} Unexpected image type.", logPrefix);
        return false;
    }

    m_inputWidth = firstImage.cols;
    m_inputHeight = firstImage.rows;
    // yuv420p requires even dimensions
    m_width = m_isDepth ? m_inputWidth : (m_inputWidth & ~1);
    m_height = m_isDepth ? m_inputHeight : (m_inputHeight & ~1);
    if (m_width <= 0 || m_height <= 0)
    {
        log()->error("{} Invalid image size {}x{}.", logPrefix, m_inputWidth, m_inputHeight);
        return false;
    }

    const std::string fileName = path.string();
    int ret = avformat_alloc_output_context2(&m_format, nullptr, nullptr, fileName.c_str());
    if (ret < 0 || m_format == nullptr)
    {
        log()->error("{} Unable to create the output {}. Error: {}.",
                     logPrefix,
                     fileName,
                     errorString(ret));
        this->close();
        return false;
    }

    std::vector<std::string> encoders;
    if (m_isDepth)
    {
        encoders = {"ffv1"};
    } else if (!m_options.encoder.empty())
    {
        encoders = {m_options.encoder};
    } else
    {
        encoders = {"libx264", "libopenh264", "mpeg4"};
    }

    const auto encoder = std::find_if(encoders.begin(), encoders.end(), [this](const auto& name) {
        return this->openEncoder(name);
    });
    if (encoder == encoders.end())
    {
        log()->error("{} None of the encoders is available.", logPrefix);
        this->close();
        return false;
    }

    m_stream = avformat_new_stream(m_format, nullptr);
    if (m_stream == nullptr
        || avcodec_parameters_from_context(m_stream->codecpar, m_codec) < 0)
    {
        log()->error("{} Unable to create the video stream.", logPrefix);
        this->close();
        return false;
    }
    m_stream->time_base = m_codec->time_base;

    std::ostringstream firstTime;
    firstTime.precision(17);
    firstTime << firstImageTime;
    av_dict_set(&m_format->metadata, "blf_first_frame_time", firstTime.str().c_str(), 0);

    if (!(m_format->oformat->flags & AVFMT_NOFILE))
    {
        ret = avio_open(&m_format->pb, fileName.c_str(), AVIO_FLAG_WRITE);
        if (ret < 0)
        {
            log()->error("{} Unable to open {}. Error: {}.", logPrefix, fileName, errorString(ret));
            this->close();
            return false;
        }
    }

    AVDictionary* muxerOptions = nullptr;
    if (!m_isDepth)
    {
        // fragmented mp4 is readable also if the file is not properly closed
        av_dict_set(&muxerOptions,
                    "movflags",
                    "frag_keyframe+empty_moov+default_base_moof+use_metadata_tags",
                    0);
    }
    ret = avformat_write_header(m_format, &muxerOptions);
    av_dict_free(&muxerOptions);
    if (ret < 0)
    {
        log()->error("{} Unable to write the header of {}. Error: {}.",
                     logPrefix,
                     fileName,
                     errorString(ret));
        this->close();
        return false;
    }
    m_headerWritten = true;

    m_scaler = sws_getContext(m_inputWidth,
                              m_inputHeight,
                              m_isDepth ? AV_PIX_FMT_GRAY16 : AV_PIX_FMT_BGR24,
                              m_width,
                              m_height,
                              m_codec->pix_fmt,
                              SWS_BILINEAR,
                              nullptr,
                              nullptr,
                              nullptr);
    m_frame = av_frame_alloc();
    m_packet = av_packet_alloc();
    if (m_scaler == nullptr || m_frame == nullptr || m_packet == nullptr)
    {
        log()->error("{} Unable to allocate the conversion buffers.", logPrefix);
        this->close();
        return false;
    }

    m_frame->format = m_codec->pix_fmt;
    m_frame->width = m_width;
    m_frame->height = m_height;
    if (av_frame_get_buffer(m_frame, 0) < 0)
    {
        log()->error("{} Unable to allocate the frame.", logPrefix);
        this->close();
        return false;
    }

    m_firstTime = firstImageTime;
    m_lastPts = -1;

    log()->info("{} Recording {} with the encoder {}.", logPrefix, fileName, *encoder);
    return true;
}

bool FFmpegVideoWriter::encode(AVFrame* frame)
{
    constexpr auto logPrefix = "[FFmpegVideoWriter::encode]";

    int ret = avcodec_send_frame(m_codec, frame);
    if (ret < 0)
    {
        log()->error("{} Unable to send the frame to the encoder. Error: {}.",
                     logPrefix,
                     errorString(ret));
        return false;
    }

    while (true)
    {
        ret = avcodec_receive_packet(m_codec, m_packet);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        {
            return true;
        }
        if (ret < 0)
        {
            log()->error("{} Unable to encode the frame. Error: {}.", logPrefix, errorString(ret));
            return false;
        }

        av_packet_rescale_ts(m_packet, m_codec->time_base, m_stream->time_base);
        m_packet->stream_index = m_stream->index;
        ret = av_interleaved_write_frame(m_format, m_packet);
        if (ret < 0)
        {
            log()->error("{} Unable to write the packet. Error: {}.", logPrefix, errorString(ret));
            return false;
        }
    }
}

bool FFmpegVideoWriter::write(const cv::Mat& image, double time)
{
    if (m_codec == nullptr || m_frame == nullptr)
    {
        return false;
    }

    if (image.cols != m_inputWidth || image.rows != m_inputHeight
        || image.type() != (m_isDepth ? CV_16UC1 : CV_8UC3))
    {
        log()->error("[FFmpegVideoWriter::write] The image size or type changed while recording.");
        return false;
    }

    // the presentation timestamps must be strictly increasing
    std::int64_t pts = std::llround((time - m_firstTime) * timeBase.den / timeBase.num);
    pts = std::max(pts, m_lastPts + 1);

    if (av_frame_make_writable(m_frame) < 0)
    {
        return false;
    }

    const std::uint8_t* source[1] = {image.data};
    const int sourceStride[1] = {static_cast<int>(image.step[0])};
    sws_scale(m_scaler, source, sourceStride, 0, image.rows, m_frame->data, m_frame->linesize);
    m_frame->pts = pts;

    if (!this->encode(m_frame))
    {
        return false;
    }

    m_lastPts = pts;
    return true;
}

void FFmpegVideoWriter::close()
{
    if (m_headerWritten)
    {
        // flush the frames buffered by the encoder
        if (m_packet != nullptr)
        {
            this->encode(nullptr);
        }
        av_write_trailer(m_format);
        m_headerWritten = false;
    }

    if (m_format != nullptr && !(m_format->oformat->flags & AVFMT_NOFILE))
    {
        avio_closep(&m_format->pb);
    }
    avformat_free_context(m_format);
    m_format = nullptr;
    m_stream = nullptr;

    avcodec_free_context(&m_codec);
    av_frame_free(&m_frame);
    av_packet_free(&m_packet);
    sws_freeContext(m_scaler);
    m_scaler = nullptr;
}
