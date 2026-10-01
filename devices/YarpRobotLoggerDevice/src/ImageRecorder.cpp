/**
 * @file ImageRecorder.cpp
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <filesystem>
#include <future>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/dict.h>
#include <libavutil/frame.h>
#include <libswscale/swscale.h>
}

#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotLogger/ImageRecorder.h>

using namespace BipedalLocomotion::RobotLogger;
using namespace BipedalLocomotion;

namespace
{

struct VideoEncoderOptions
{
    std::string encoder; /**< FFmpeg encoder. If empty the first available is used. */
    double fps{30.0}; /**< Nominal frame rate. */
};

/**
 * IImageWriter stores a sequence of 8-bit BGR (CV_8UC3) or 16-bit (CV_16UC1) images.
 */
class IImageWriter
{
public:
    virtual ~IImageWriter() = default;

    /** Extension of the file. It is empty if the images are saved in a folder. */
    virtual std::string extension() const = 0;

    virtual bool
    open(const std::filesystem::path& path, const cv::Mat& firstImage, double firstImageTime) = 0;

    virtual bool write(const cv::Mat& image, double time) = 0;

    virtual void close() = 0;
};

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
            log()->error("[FramesWriter::open] Unable to create the folder {}. Error: {}.",
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

std::string ffmpegError(int error)
{
    char buffer[AV_ERROR_MAX_STRING_SIZE] = {0};
    av_strerror(error, buffer, sizeof(buffer));
    return buffer;
}

struct FormatDeleter
{
    void operator()(AVFormatContext* format) const
    {
        if (!(format->oformat->flags & AVFMT_NOFILE))
        {
            avio_closep(&format->pb);
        }
        avformat_free_context(format);
    }
};

struct CodecDeleter
{
    void operator()(AVCodecContext* codec) const
    {
        avcodec_free_context(&codec);
    }
};

struct FrameDeleter
{
    void operator()(AVFrame* frame) const
    {
        av_frame_free(&frame);
    }
};

struct PacketDeleter
{
    void operator()(AVPacket* packet) const
    {
        av_packet_free(&packet);
    }
};

struct ScalerDeleter
{
    void operator()(SwsContext* scaler) const
    {
        sws_freeContext(scaler);
    }
};

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
    FFmpegVideoWriter(bool isDepth, VideoEncoderOptions options)
        : m_isDepth(isDepth)
        , m_options(std::move(options))
    {
    }

    ~FFmpegVideoWriter() override
    {
        this->close();
    }

    std::string extension() const final
    {
        return m_isDepth ? ".mkv" : ".mp4";
    }

    bool
    open(const std::filesystem::path& path, const cv::Mat& firstImage, double firstImageTime) final;

    bool write(const cv::Mat& image, double time) final;

    void close() final;

private:
    // the timestamps are expressed in milliseconds
    static constexpr AVRational timeBase{1, 1000};

    bool openEncoder(const std::string& name);

    /** Encode the current frame or, if flush is true, drain the encoder. */
    bool encode(bool flush);

    bool m_isDepth;
    VideoEncoderOptions m_options;

    std::unique_ptr<AVFormatContext, FormatDeleter> m_format;
    std::unique_ptr<AVCodecContext, CodecDeleter> m_codec;
    std::unique_ptr<AVFrame, FrameDeleter> m_frame;
    std::unique_ptr<AVPacket, PacketDeleter> m_packet;
    std::unique_ptr<SwsContext, ScalerDeleter> m_scaler;
    int m_streamIndex{-1}; /**< The stream is owned by m_format. */
    bool m_headerWritten{false};

    int m_inputWidth{0};
    int m_inputHeight{0};
    int m_width{0};
    int m_height{0};
    double m_firstTime{0};
    std::int64_t m_lastPts{-1};
};

bool FFmpegVideoWriter::openEncoder(const std::string& name)
{
    const AVCodec* codec = avcodec_find_encoder_by_name(name.c_str());
    if (codec == nullptr)
    {
        return false;
    }

    std::unique_ptr<AVCodecContext, CodecDeleter> context(avcodec_alloc_context3(codec));
    if (context == nullptr)
    {
        return false;
    }
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

    const int ret = avcodec_open2(context.get(), codec, &options);
    av_dict_free(&options);
    if (ret < 0)
    {
        log()->warn("[FFmpegVideoWriter::openEncoder] Unable to open the encoder {}. Error: {}.",
                    name,
                    ffmpegError(ret));
        return false;
    }

    m_codec = std::move(context);
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
    AVFormatContext* format = nullptr;
    int ret = avformat_alloc_output_context2(&format, nullptr, nullptr, fileName.c_str());
    m_format.reset(format);
    if (ret < 0 || m_format == nullptr)
    {
        log()->error("{} Unable to create the output {}. Error: {}.",
                     logPrefix,
                     fileName,
                     ffmpegError(ret));
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

    AVStream* stream = avformat_new_stream(m_format.get(), nullptr);
    if (stream == nullptr || avcodec_parameters_from_context(stream->codecpar, m_codec.get()) < 0)
    {
        log()->error("{} Unable to create the video stream.", logPrefix);
        this->close();
        return false;
    }
    stream->time_base = m_codec->time_base;
    m_streamIndex = stream->index;

    std::ostringstream firstTime;
    firstTime.precision(17);
    firstTime << firstImageTime;
    av_dict_set(&m_format->metadata, "blf_first_frame_time", firstTime.str().c_str(), 0);

    if (!(m_format->oformat->flags & AVFMT_NOFILE))
    {
        ret = avio_open(&m_format->pb, fileName.c_str(), AVIO_FLAG_WRITE);
        if (ret < 0)
        {
            log()->error("{} Unable to open {}. Error: {}.", logPrefix, fileName, ffmpegError(ret));
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
    ret = avformat_write_header(m_format.get(), &muxerOptions);
    av_dict_free(&muxerOptions);
    if (ret < 0)
    {
        log()->error("{} Unable to write the header of {}. Error: {}.",
                     logPrefix,
                     fileName,
                     ffmpegError(ret));
        this->close();
        return false;
    }
    m_headerWritten = true;

    m_scaler.reset(sws_getContext(m_inputWidth,
                                  m_inputHeight,
                                  m_isDepth ? AV_PIX_FMT_GRAY16 : AV_PIX_FMT_BGR24,
                                  m_width,
                                  m_height,
                                  m_codec->pix_fmt,
                                  SWS_BILINEAR,
                                  nullptr,
                                  nullptr,
                                  nullptr));
    m_frame.reset(av_frame_alloc());
    m_packet.reset(av_packet_alloc());
    if (m_scaler == nullptr || m_frame == nullptr || m_packet == nullptr)
    {
        log()->error("{} Unable to allocate the conversion buffers.", logPrefix);
        this->close();
        return false;
    }

    m_frame->format = m_codec->pix_fmt;
    m_frame->width = m_width;
    m_frame->height = m_height;
    if (av_frame_get_buffer(m_frame.get(), 0) < 0)
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

bool FFmpegVideoWriter::encode(bool flush)
{
    constexpr auto logPrefix = "[FFmpegVideoWriter::encode]";

    int ret = avcodec_send_frame(m_codec.get(), flush ? nullptr : m_frame.get());
    if (ret < 0)
    {
        log()->error("{} Unable to send the frame to the encoder. Error: {}.",
                     logPrefix,
                     ffmpegError(ret));
        return false;
    }

    const AVStream& stream = *m_format->streams[m_streamIndex];
    while (true)
    {
        ret = avcodec_receive_packet(m_codec.get(), m_packet.get());
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        {
            return true;
        }
        if (ret < 0)
        {
            log()->error("{} Unable to encode the frame. Error: {}.", logPrefix, ffmpegError(ret));
            return false;
        }

        av_packet_rescale_ts(m_packet.get(), m_codec->time_base, stream.time_base);
        m_packet->stream_index = m_streamIndex;
        ret = av_interleaved_write_frame(m_format.get(), m_packet.get());
        if (ret < 0)
        {
            log()->error("{} Unable to write the packet. Error: {}.", logPrefix, ffmpegError(ret));
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

    if (av_frame_make_writable(m_frame.get()) < 0)
    {
        return false;
    }

    const std::uint8_t* source[1] = {image.data};
    const int sourceStride[1] = {static_cast<int>(image.step[0])};
    sws_scale(m_scaler.get(), source, sourceStride, 0, image.rows, m_frame->data, m_frame->linesize);
    m_frame->pts = pts;

    if (!this->encode(false))
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
            this->encode(true);
        }
        av_write_trailer(m_format.get());
        m_headerWritten = false;
    }

    m_format.reset();
    m_streamIndex = -1;
    m_codec.reset();
    m_frame.reset();
    m_packet.reset();
    m_scaler.reset();
}

} // namespace

struct ImageRecorder::Impl
{
    enum class CommandType
    {
        Image,
        Rotate,
        Discard,
        Sync,
        Stop
    };

    struct Command
    {
        CommandType type{CommandType::Sync};
        cv::Mat image;
        double time{0.0};
        std::string fileName;
        std::shared_ptr<std::promise<void>> done;
    };

    std::string name;
    std::string imageType;
    std::string channel;
    double fps{0.0};

    std::unique_ptr<IImageSource> source;
    std::shared_ptr<TelemetryBuffer> buffer;

    std::thread acquisitionThread;
    std::atomic<bool> acquisitionRunning{false};
    std::atomic<std::size_t> droppedImages{0};

    std::thread writerThread;
    std::mutex queueMutex;
    std::condition_variable queueCv;
    std::deque<Command> queue;
    std::size_t queuedImages{0};
    std::size_t maxQueuedImages{0};

    // accessed only by the writer thread
    std::unique_ptr<IImageWriter> writer;
    bool fileOpen{false};
    bool writerFailed{false};
    unsigned int imageIndex{0};

    std::filesystem::path temporaryPath() const
    {
        return buffer->getLogFolder() / ("output_" + name + "_" + imageType + writer->extension());
    }

    void sendAndWait(Command command)
    {
        if (!writerThread.joinable())
        {
            return;
        }

        command.done = std::make_shared<std::promise<void>>();
        auto done = command.done->get_future();
        {
            std::lock_guard lock(queueMutex);
            queue.push_back(std::move(command));
        }
        queueCv.notify_one();
        done.wait();
    }

    void enqueueImage(cv::Mat&& image, double time)
    {
        {
            std::lock_guard lock(queueMutex);
            if (queuedImages < maxQueuedImages)
            {
                queue.push_back({CommandType::Image, std::move(image), time});
                queuedImages++;
                queueCv.notify_one();
                return;
            }
        }

        // log only a few messages
        const std::size_t dropped = ++droppedImages;
        if ((dropped & (dropped - 1)) == 0)
        {
            log()->warn("[ImageRecorder::enqueueImage] The images of {} ({}) are acquired faster "
                        "than they are written. Dropped images: {}.",
                        name,
                        imageType,
                        dropped);
        }
    }

    void acquisitionLoop()
    {
        using namespace std::chrono_literals;

        const bool polling = fps > 0;
        const auto period = polling ? std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::duration<double>(1.0 / fps))
                                    : std::chrono::nanoseconds(0);

        auto wakeUpTime = BipedalLocomotion::clock().now();
        while (acquisitionRunning)
        {
            cv::Mat image;
            const bool ok = source->read(image);
            const auto now = BipedalLocomotion::clock().now();

            if (ok && !image.empty() && acquisitionRunning)
            {
                this->enqueueImage(std::move(image), std::chrono::duration<double>(now).count());
            }

            if (polling)
            {
                wakeUpTime += period;
                // do not catch up if late and handle clock resets
                if (wakeUpTime < now || wakeUpTime > now + period)
                {
                    wakeUpTime = now + period;
                }
                BipedalLocomotion::clock().sleepUntil(wakeUpTime);
            } else if (!ok)
            {
                BipedalLocomotion::clock().sleepFor(10ms);
            }
        }
    }

    void writerLoop()
    {
        while (true)
        {
            Command command;
            {
                std::unique_lock lock(queueMutex);
                queueCv.wait(lock, [this] { return !queue.empty(); });
                command = std::move(queue.front());
                queue.pop_front();
                if (command.type == CommandType::Image)
                {
                    queuedImages--;
                }
            }

            switch (command.type)
            {
            case CommandType::Image:
                this->writeImage(command.image, command.time);
                break;
            case CommandType::Rotate:
                this->closeFile(command.fileName);
                break;
            case CommandType::Discard:
                this->closeFile("");
                break;
            case CommandType::Stop:
                // the file is kept with its temporary name
                if (fileOpen)
                {
                    writer->close();
                    fileOpen = false;
                }
                break;
            case CommandType::Sync:
                break;
            }

            if (command.done != nullptr)
            {
                command.done->set_value();
            }

            if (command.type == CommandType::Stop)
            {
                return;
            }
        }
    }

    void writeImage(const cv::Mat& image, double time)
    {
        constexpr auto logPrefix = "[ImageRecorder::writeImage]";

        if (!fileOpen)
        {
            if (writerFailed)
            {
                return;
            }

            const auto path = this->temporaryPath();

            // a file left by a previous execution (e.g., after a crash) is kept
            if (std::filesystem::exists(path))
            {
                const std::time_t now = std::time(nullptr);
                std::ostringstream suffix;
                suffix << "_recovered_" << std::put_time(std::localtime(&now), "%Y_%m_%d_%H_%M_%S");
                auto recovered
                    = path.parent_path()
                      / (path.stem().string() + suffix.str() + path.extension().string());
                std::error_code ec;
                std::filesystem::rename(path, recovered, ec);
                log()->warn("{} Found the file {} of a previous execution. It has been renamed "
                            "as {}.",
                            logPrefix,
                            path.string(),
                            recovered.string());
            }

            if (!writer->open(path, image, time))
            {
                log()->error("{} Unable to open {}. The images of {} ({}) will not be saved until "
                             "the next file.",
                             logPrefix,
                             path.string(),
                             name,
                             imageType);
                writerFailed = true;
                return;
            }
            fileOpen = true;
            imageIndex = 0;
        }

        if (writer->write(image, time))
        {
            buffer->push(channel, imageIndex, time);
            imageIndex++;
        }
    }

    void closeFile(const std::string& fileName)
    {
        constexpr auto logPrefix = "[ImageRecorder::closeFile]";

        writerFailed = false;
        if (!fileOpen)
        {
            return;
        }

        writer->close();
        fileOpen = false;

        const auto temporary = this->temporaryPath();
        std::error_code ec;
        if (fileName.empty())
        {
            std::filesystem::remove_all(temporary, ec);
            return;
        }

        const std::filesystem::path target
            = fileName + "_" + name + "_" + imageType + writer->extension();
        if (std::filesystem::exists(target))
        {
            log()->error("{} Unable to rename {} as {}. The file already exists.",
                         logPrefix,
                         temporary.string(),
                         target.string());
            return;
        }

        std::filesystem::rename(temporary, target, ec);
        if (ec)
        {
            log()->error("{} Unable to rename {} as {}. Error: {}.",
                         logPrefix,
                         temporary.string(),
                         target.string(),
                         ec.message());
        }
    }
};

ImageRecorder::ImageRecorder()
    : m_pimpl(std::make_unique<Impl>())
{
}

ImageRecorder::~ImageRecorder()
{
    this->stop();
}

bool ImageRecorder::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                               std::unique_ptr<IImageSource> source,
                               std::shared_ptr<TelemetryBuffer> buffer)
{
    constexpr auto logPrefix = "[ImageRecorder::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr || source == nullptr || buffer == nullptr)
    {
        log()->error("{} The handler, the source and the buffer must be valid.", logPrefix);
        return false;
    }

    std::string saveMode;
    if (!ptr->getParameter("name", m_pimpl->name)
        || !ptr->getParameter("image_type", m_pimpl->imageType)
        || !ptr->getParameter("channel", m_pimpl->channel)
        || !ptr->getParameter("save_mode", saveMode))
    {
        log()->error("{} Unable to get the parameters 'name', 'image_type', 'channel' and "
                     "'save_mode'.",
                     logPrefix);
        return false;
    }

    if (m_pimpl->imageType != "rgb" && m_pimpl->imageType != "depth")
    {
        log()->error("{} The parameter 'image_type' must be either 'rgb' or 'depth'. Provided: "
                     "{}.",
                     logPrefix,
                     m_pimpl->imageType);
        return false;
    }

    if (saveMode != "video" && saveMode != "frame")
    {
        log()->error("{} The parameter 'save_mode' must be either 'video' or 'frame'. Provided: "
                     "{}.",
                     logPrefix,
                     saveMode);
        return false;
    }

    ptr->getParameter("fps", m_pimpl->fps);
    if (m_pimpl->fps < 0)
    {
        log()->error("{} The parameter 'fps' must be non negative.", logPrefix);
        return false;
    }

    VideoEncoderOptions encoder;
    encoder.fps = m_pimpl->fps > 0 ? m_pimpl->fps : encoder.fps;
    ptr->getParameter("video_encoder", encoder.encoder);

    if (saveMode == "frame")
    {
        m_pimpl->writer = std::make_unique<FramesWriter>();
    } else
    {
        const bool isDepth = m_pimpl->imageType == "depth";
        m_pimpl->writer = std::make_unique<FFmpegVideoWriter>(isDepth, encoder);
    }

    // about two seconds of images
    constexpr std::size_t minQueueSize = 10;
    const double rate = m_pimpl->fps > 0 ? m_pimpl->fps : 15.0;
    m_pimpl->maxQueuedImages
        = std::max(minQueueSize, static_cast<std::size_t>(std::ceil(2 * rate)));

    m_pimpl->source = std::move(source);
    m_pimpl->buffer = std::move(buffer);
    return true;
}

bool ImageRecorder::start()
{
    constexpr auto logPrefix = "[ImageRecorder::start]";

    if (m_pimpl->writer == nullptr)
    {
        log()->error("{} The recorder is not initialized.", logPrefix);
        return false;
    }

    if (m_pimpl->writerThread.joinable())
    {
        return true;
    }

    if (!m_pimpl->buffer->addStoredChannel(m_pimpl->channel, {1, 1}, {"frame_index"}))
    {
        log()->error("{} Unable to add the channel {}.", logPrefix, m_pimpl->channel);
        return false;
    }

    m_pimpl->droppedImages = 0;
    m_pimpl->source->resume();
    m_pimpl->writerThread = std::thread([this] { m_pimpl->writerLoop(); });
    m_pimpl->acquisitionRunning = true;
    m_pimpl->acquisitionThread = std::thread([this] { m_pimpl->acquisitionLoop(); });
    return true;
}

void ImageRecorder::stopAcquisition()
{
    if (!m_pimpl->acquisitionThread.joinable())
    {
        return;
    }

    m_pimpl->acquisitionRunning = false;
    m_pimpl->source->interrupt();
    m_pimpl->acquisitionThread.join();

    m_pimpl->sendAndWait({Impl::CommandType::Sync});
}

void ImageRecorder::rotate(const std::string& fileName)
{
    m_pimpl->sendAndWait({Impl::CommandType::Rotate, {}, 0.0, fileName});
}

void ImageRecorder::discard()
{
    m_pimpl->sendAndWait({Impl::CommandType::Discard});
}

void ImageRecorder::stop()
{
    this->stopAcquisition();

    if (m_pimpl->writerThread.joinable())
    {
        m_pimpl->sendAndWait({Impl::CommandType::Stop});
        m_pimpl->writerThread.join();
    }
}
