/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotLogger/ImageRecorder.h>

using namespace BipedalLocomotion::RobotLogger;

ImageRecorder::ImageRecorder(ImageRecorderOptions options,
                             std::unique_ptr<IImageSource> source,
                             DataStorage& storage)
    : m_options(std::move(options))
    , m_source(std::move(source))
    , m_storage(storage)
{
    m_writer = createImageWriter(m_options.saveMode, m_options.isDepth, m_options.encoder);

    // about two seconds of images
    constexpr std::size_t minQueueSize = 10;
    const double rate = m_options.fps > 0 ? m_options.fps : 15.0;
    m_maxQueuedImages = std::max(minQueueSize, static_cast<std::size_t>(std::ceil(2 * rate)));
}

ImageRecorder::~ImageRecorder()
{
    this->stop();
}

std::filesystem::path ImageRecorder::temporaryPath() const
{
    return m_storage.logFolder()
           / ("output_" + m_options.name + "_" + m_options.imageType + m_writer->extension());
}

bool ImageRecorder::start()
{
    constexpr auto logPrefix = "[ImageRecorder::start]";

    if (m_writerThread.joinable())
    {
        return true;
    }

    if (!m_storage.addChannel(m_options.channel, {{1, 1}, {"frame_index"}}))
    {
        log()->error("{} Unable to add the channel {}.", logPrefix, m_options.channel);
        return false;
    }

    m_droppedImages = 0;
    m_source->resume();
    m_writerThread = std::thread([this] { this->writerLoop(); });
    m_acquisitionRunning = true;
    m_acquisitionThread = std::thread([this] { this->acquisitionLoop(); });
    return true;
}

void ImageRecorder::stopAcquisition()
{
    if (!m_acquisitionThread.joinable())
    {
        return;
    }

    m_acquisitionRunning = false;
    m_source->interrupt();
    m_acquisitionThread.join();

    this->sendAndWait({CommandType::Sync});
}

void ImageRecorder::rotate(const std::string& fileName)
{
    this->sendAndWait({CommandType::Rotate, {}, 0.0, fileName});
}

void ImageRecorder::discard()
{
    this->sendAndWait({CommandType::Discard});
}

void ImageRecorder::stop()
{
    this->stopAcquisition();

    if (m_writerThread.joinable())
    {
        this->sendAndWait({CommandType::Stop});
        m_writerThread.join();
    }
}

void ImageRecorder::sendAndWait(Command command)
{
    if (!m_writerThread.joinable())
    {
        return;
    }

    command.done = std::make_shared<std::promise<void>>();
    auto done = command.done->get_future();
    {
        std::lock_guard lock(m_queueMutex);
        m_queue.push_back(std::move(command));
    }
    m_queueCv.notify_one();
    done.wait();
}

void ImageRecorder::enqueueImage(cv::Mat&& image, double time)
{
    {
        std::lock_guard lock(m_queueMutex);
        if (m_queuedImages < m_maxQueuedImages)
        {
            m_queue.push_back({CommandType::Image, std::move(image), time});
            m_queuedImages++;
            m_queueCv.notify_one();
            return;
        }
    }

    // log only a few messages
    const std::size_t dropped = ++m_droppedImages;
    if ((dropped & (dropped - 1)) == 0)
    {
        log()->warn("[ImageRecorder::enqueueImage] The images of {} ({}) are acquired faster "
                    "than they are written. Dropped images: {}.",
                    m_options.name,
                    m_options.imageType,
                    dropped);
    }
}

void ImageRecorder::acquisitionLoop()
{
    using namespace std::chrono_literals;

    const bool polling = m_options.fps > 0;
    const auto period = polling ? std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::duration<double>(1.0 / m_options.fps))
                                : std::chrono::nanoseconds(0);

    auto wakeUpTime = BipedalLocomotion::clock().now();
    while (m_acquisitionRunning)
    {
        cv::Mat image;
        const bool ok = m_source->read(image);
        const auto now = BipedalLocomotion::clock().now();

        if (ok && !image.empty() && m_acquisitionRunning)
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

void ImageRecorder::writerLoop()
{
    while (true)
    {
        Command command;
        {
            std::unique_lock lock(m_queueMutex);
            m_queueCv.wait(lock, [this] { return !m_queue.empty(); });
            command = std::move(m_queue.front());
            m_queue.pop_front();
            if (command.type == CommandType::Image)
            {
                m_queuedImages--;
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
            if (m_fileOpen)
            {
                m_writer->close();
                m_fileOpen = false;
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

void ImageRecorder::writeImage(const cv::Mat& image, double time)
{
    constexpr auto logPrefix = "[ImageRecorder::writeImage]";

    if (!m_fileOpen)
    {
        if (m_writerFailed)
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
            auto recovered = path.parent_path()
                             / (path.stem().string() + suffix.str() + path.extension().string());
            std::error_code ec;
            std::filesystem::rename(path, recovered, ec);
            log()->warn("{} Found the file {} of a previous execution. It has been renamed as {}.",
                        logPrefix,
                        path.string(),
                        recovered.string());
        }

        if (!m_writer->open(path, image, time))
        {
            log()->error("{} Unable to open {}. The images of {} ({}) will not be saved until "
                         "the next file.",
                         logPrefix,
                         path.string(),
                         m_options.name,
                         m_options.imageType);
            m_writerFailed = true;
            return;
        }
        m_fileOpen = true;
        m_imageIndex = 0;
    }

    if (m_writer->write(image, time))
    {
        m_storage.push(m_options.channel, m_imageIndex, time);
        m_imageIndex++;
    }
}

void ImageRecorder::closeFile(const std::string& fileName)
{
    constexpr auto logPrefix = "[ImageRecorder::closeFile]";

    m_writerFailed = false;
    if (!m_fileOpen)
    {
        return;
    }

    m_writer->close();
    m_fileOpen = false;

    const auto temporary = this->temporaryPath();
    std::error_code ec;
    if (fileName.empty())
    {
        std::filesystem::remove_all(temporary, ec);
        return;
    }

    const std::filesystem::path target = fileName + "_" + m_options.name + "_"
                                         + m_options.imageType + m_writer->extension();
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
