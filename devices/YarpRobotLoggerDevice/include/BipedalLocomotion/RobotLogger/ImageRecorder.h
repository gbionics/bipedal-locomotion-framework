/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_IMAGE_RECORDER_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_IMAGE_RECORDER_H

#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

#include <BipedalLocomotion/RobotLogger/DataStorage.h>
#include <BipedalLocomotion/RobotLogger/ImageWriters.h>

namespace BipedalLocomotion
{
namespace RobotLogger
{

class IImageSource
{
public:
    virtual ~IImageSource() = default;

    /**
     * Read an image. A source may block until a new image is available.
     * @param image the image. It must own its data.
     */
    virtual bool read(cv::Mat& image) = 0;

    /** Unblock a pending read. */
    virtual void interrupt()
    {
    }

    /** Allow the reads again after interrupt(). */
    virtual void resume()
    {
    }
};

struct ImageRecorderOptions
{
    std::string name; /**< Name used in the file names, e.g., the camera name. */
    std::string imageType; /**< rgb or depth. */
    std::string channel; /**< Channel storing the index and the time of each saved image. */
    ImageSaveMode saveMode{ImageSaveMode::Video};
    bool isDepth{false}; /**< True if the images are CV_16UC1, CV_8UC3 otherwise. */
    double fps{0.0}; /**< Acquisition rate. If zero, the images are read as soon as they arrive. */
    VideoEncoderOptions encoder;
};

/**
 * ImageRecorder saves the images provided by a source.
 *
 * The images are read by an acquisition thread and written by a writer thread, so that a slow
 * encoding does not affect the acquisition. If the writer thread cannot keep up, the new images
 * are dropped.
 *
 * The images are written in a temporary file (or folder) in the log folder. When rotate() is
 * called, the file is closed and renamed as `<fileName>_<name>_<imageType><extension>`, and the
 * next images are written in a new file.
 *
 * For each saved image, its index in the file and its time are pushed in the storage channel.
 * The index restarts from zero in every file, hence the time of the first image of the file
 * `<fileName>_...` is the time associated to the index 0 in `<fileName>.mat`.
 */
class ImageRecorder
{
public:
    ImageRecorder(ImageRecorderOptions options,
                  std::unique_ptr<IImageSource> source,
                  DataStorage& storage);

    ~ImageRecorder();

    /** Add the storage channel and start the acquisition. */
    bool start();

    /** Stop the acquisition and wait until all the acquired images are written. */
    void stopAcquisition();

    /** Close the current file and rename it with the given file name (without extension). */
    void rotate(const std::string& fileName);

    /** Close and delete the current file. */
    void discard();

    /** Stop the acquisition and the writer. */
    void stop();

private:
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

    void acquisitionLoop();
    void writerLoop();
    void enqueueImage(cv::Mat&& image, double time);
    void sendAndWait(Command command);
    void writeImage(const cv::Mat& image, double time);
    void closeFile(const std::string& fileName);
    std::filesystem::path temporaryPath() const;

    ImageRecorderOptions m_options;
    std::unique_ptr<IImageSource> m_source;
    DataStorage& m_storage;

    std::thread m_acquisitionThread;
    std::atomic<bool> m_acquisitionRunning{false};
    std::atomic<std::size_t> m_droppedImages{0};

    std::thread m_writerThread;
    std::mutex m_queueMutex;
    std::condition_variable m_queueCv;
    std::deque<Command> m_queue;
    std::size_t m_queuedImages{0};
    std::size_t m_maxQueuedImages{0};

    // accessed only by the writer thread
    std::unique_ptr<IImageWriter> m_writer;
    bool m_fileOpen{false};
    bool m_writerFailed{false};
    unsigned int m_imageIndex{0};
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_IMAGE_RECORDER_H
