#include <BipedalLocomotion/Conversions/CommonConversions.h>
#include <BipedalLocomotion/GenericContainer/Vector.h>
#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::GenericContainer;
using namespace BipedalLocomotion::Perception;
using namespace BipedalLocomotion::Conversions;

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#ifndef __cpp_lib_optional // Check if std::optional is available
#include <memory> // For std::unique_ptr

template <typename T>
class Optional
{
public:
    Optional() : m_hasValue(false) {}
    Optional(const T& value) : m_value(value), m_hasValue(true) {}

    bool has_value() const { return m_hasValue; }
    T value() const
    {
        if (!m_hasValue)
        {
            throw std::runtime_error("Accessing an empty Optional value");
        }
        return m_value;
    }

private:
    bool m_hasValue;
    T m_value;
};

#else // std::optional is available
#include <optional>
using Optional = std::optional;
#endif

class ArucoDetector::Impl
{
public:
    /**
     * clear all internal buffers
     */
    void resetBuffers();

    // parameters
    bool isBoard{false}; /**< flag to check if we want to detect a board */
    cv::Ptr<cv::aruco::Dictionary> dictionary; /**< container with detected markers data */
    double markerLength; /**< marker length*/
    cv::Mat cameraMatrix; /**< camera calibration matrix*/
    cv::Mat distCoeff; /**< camera distortion coefficients*/
    cv::Ptr<cv::aruco::Board> board; /**< ArUco board configuration */

    ArucoDetectorOutput out; /**< container with detected markers data */
    cv::Mat currentImg; /**< currently set image */
    double currentTime{-1.0}; /** time at which currentImg was set */
    std::vector<int> currentDetectedMarkerIds; /**< currently detected marker ids */
    std::vector<std::vector<cv::Point2f>> currentDetectedMarkerCorners; /**< currently detected
                                                                           marker corners */

    std::vector<cv::Vec3d> currentDetectedMarkersRotVecs; /**< currently detected marker
                                                               rotation vector */
    std::vector<cv::Vec3d> currentDetectedMarkersTransVecs; /**< currently detected marker
                                                               translation vector */

    bool initialized{false}; /**< flag to check if detector was initialized properly */

    cv::Mat R; /**< placeholder rotation matrix as cv Mat object*/
    Eigen::Matrix3d Reig; /**< placeholder rotation matrix as Eigen object*/
    Eigen::Vector3d teig; /**< placeholder translation vector as Eigen object*/
    Eigen::Matrix4d poseEig; /**< placeholder pose matrix as Eigen object*/

     // Optional board parameters
    Optional<int> markersX;
    Optional<int> markersY;
    Optional<double> markerSeparation;

    /**
     * Utility map for choosing Aruco marker dictionary depending on user parameter
     */
    // TODO(traversaro): when we drop support for OpenCV < 4.7.0, we can cleanup this part
    // we can also check if there are other dictionary that should be added here
#if (CV_VERSION_MAJOR >= 5) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#define PREDEFINED_DICTIONARY_NAME PredefinedDictionaryType
#endif
    std::unordered_map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME>
        availableDict{{"4X4_50", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50},
                      {"4X4_100", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100},
                      {"4X4_250", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_250},
                      {"4X4_1000", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_1000},
                      {"5X5_50", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_50},
                      {"5X5_100", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_100},
                      {"5X5_250", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_250},
                      {"5X5_1000", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_1000},
                      {"6X6_50", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50},
                      {"6X6_100", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_100},
                      {"6X6_250", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250},
                      {"6X6_1000", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_1000},
                      {"7X7_50", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_50},
                      {"7X7_100", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_100},
                      {"7X7_250", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_250},
                      {"7X7_1000", cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_1000},
                      {"ARUCO_ORIGINAL",
                       cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL}};

    // Helper method for single marker pose estimation
    void estimateSingleMarkersPose();
};

ArucoDetector::ArucoDetector()
    : m_pimpl(std::make_unique<Impl>())
{
}

ArucoDetector::~ArucoDetector() = default;

bool ArucoDetector::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto printPrefix = "[ArucoDetector::initialize]";
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        log()->error("{} The parameter handler has expired. Please check its scope.", printPrefix);
        return false;
    }

    std::string dictName;
    if (!handle->getParameter("marker_dictionary", dictName))
    {
        log()->error("{} The parameter handler could not find \" marker_dictionary \" in the "
                     "configuration file.",
                     printPrefix);
        return false;
    }

    if (m_pimpl->availableDict.find(dictName) == m_pimpl->availableDict.end())
    {
        log()->error("{} Undefined value set to \" marker_dictionary \" in the configuration file. "
                     "Please choose one among \n available options: \"4X4_50\", \"4X4_100\", "
                     "\"4X4_250\", \"4X4_1000\", \n \"5X5_50\", \"5X5_100\", \"5X5_250\", "
                     "\"5X5_1000\", \n \"6X6_50\", \"6X6_100\", \"6X6_250\", \"6X6_1000\", \n "
                     "\"7X7_50\", \"7X7_100\", \"7X7_250\", \"7X7_1000\", \n \"ARUCO_ORIGINAL\", "
                     "\n options coherent with v3.4.0 in "
                     "https://docs.opencv.org/3.4.1/d9/d6a/group__aruco.html",
                     printPrefix);
        return false;
    }

// In OpenCV 4.7.0 getPredefinedDictionary started returning a cv::aruco::Dictionary
#if (CV_VERSION_MAJOR >= 5) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    m_pimpl->dictionary = cv::makePtr<cv::aruco::Dictionary>();
    *(m_pimpl->dictionary)
        = cv::aruco::getPredefinedDictionary(m_pimpl->availableDict.at(dictName));
#else
    m_pimpl->dictionary = cv::aruco::getPredefinedDictionary(m_pimpl->availableDict.at(dictName));
#endif

    if (!handle->getParameter("is_board", m_pimpl->isBoard))
    {
        log()->error("{} The parameter handler could not find \" is_board \" in the "
                     "configuration file.",
                     printPrefix);
        return false;
    }

    if (!handle->getParameter("marker_length", m_pimpl->markerLength))
    {
        log()->error("{} The parameter handler could not find \" marker_length \" in the "
                     "configuration file.",
                     printPrefix);
        return false;
    }

    Eigen::Matrix<double, 9, 1> calibVec;
    if (!handle->getParameter("camera_matrix", calibVec))
    {
        log()->error("{} The parameter handler could not find \" camera_matrix \" in the "
                     "configuration file.",
                     printPrefix);
        return false;
    }

    Eigen::Matrix3d calibMat
        = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(calibVec.data());
    m_pimpl->cameraMatrix = cv::Mat(3, 3, CV_64F);
    cv::eigen2cv(calibMat, m_pimpl->cameraMatrix);

    Eigen::Matrix<double, 5, 1> distCoeffVec;
    if (!handle->getParameter("distortion_coefficients", distCoeffVec))
    {
        log()->error("{} The parameter handler could not find \" distortion_coefficients \" in the "
                     "configuration file.",
                     printPrefix);
        return false;
    }

    m_pimpl->distCoeff = cv::Mat(5, 1, CV_64F);
    cv::eigen2cv(distCoeffVec, m_pimpl->distCoeff);

    // Read optional board parameters
    int markersX;
    int markersY;
    double markerSeparation;

    if (handle->getParameter("markers_x", markersX))
    {
        m_pimpl->markersX = markersX;
    }

    if (handle->getParameter("markers_y", markersY))
    {
        m_pimpl->markersY = markersY;
    }

    if (handle->getParameter("marker_separation", markerSeparation))
    {
        m_pimpl->markerSeparation = markerSeparation;
    }

    // Board configuration
    if (m_pimpl->isBoard)
    {
        if (!m_pimpl->markersX.has_value() || !m_pimpl->markersY.has_value() || !m_pimpl->markerSeparation.has_value())
        {
            log()->error("{} is_board is true, but not all board parameters (markers_x, markers_y, marker_separation) were provided.", printPrefix);
            return false;
        }

        // Create the board object
        cv::Size boardSize(m_pimpl->markersX.value(), m_pimpl->markersY.value());
        float markerLength = m_pimpl->markerLength;
        float markerDistance = m_pimpl->markerSeparation.value(); // distance in meters

        // Create the board layout (assuming sequential IDs)
        std::vector<int> markerIds;
        for (int i = 0; i < boardSize.width * boardSize.height; ++i)
        {
            markerIds.push_back(i); // Sequential IDs
        }
        // Define the board using the layout and the dictionary
        // m_pimpl->board = cv::aruco::GridBoard::create(boardSize.width,
        //                                               boardSize.height,
        //                                               markerLength,
        //                                               markerDistance,
        //                                               m_pimpl->dictionary);
        m_pimpl->board = cv::makePtr<cv::aruco::GridBoard>(
                                                        cv::Size(boardSize.width, boardSize.height),
                                                        markerLength,
                                                        markerDistance,
                                                        *m_pimpl->dictionary); 
    }
    else
    {
        m_pimpl->board = nullptr; // Ensure board is null if not using it
    }

    m_pimpl->initialized = true;
    return true;
}

bool ArucoDetector::setImage(const cv::Mat& inputImg, double timeNow)
{
    constexpr auto printPrefix = "[ArucoDetector::setImage]";
    if (!m_pimpl->initialized)
    {
        log()->error("{} Unable to set image. Please initialize the ArucoDetector before setting "
                     "the image.",
                     printPrefix);
        return false;
    }

    inputImg.copyTo(m_pimpl->currentImg);
    m_pimpl->currentTime = timeNow;

    return true;
}

bool ArucoDetector::advance()
{
    constexpr auto printPrefix = "[ArucoDetector::advance]";
    if (!m_pimpl->initialized)
    {
        log()->error("{} Unable to run advance(). Please initialize the ArucoDetector first.",
                     printPrefix);
        return false;
    }

    if (m_pimpl->currentImg.empty())
    {
        log()->error("{} Unable to run advance(). Please set an image first.", printPrefix);
        return false;
    }

    m_pimpl->resetBuffers();

    cv::aruco::detectMarkers(m_pimpl->currentImg,
                             m_pimpl->dictionary,
                             m_pimpl->currentDetectedMarkerCorners,
                             m_pimpl->currentDetectedMarkerIds);

    if (m_pimpl->isBoard)
    {
        if (m_pimpl->board == nullptr) {
            log()->error("{} Board is enabled, but board object is null. This should not happen. Check initialize method.", printPrefix);
            return false; // Or handle the error in another appropriate way.
        }

        // Estimate board pose
        cv::Vec3d rvec, tvec;
        int valid_corners = cv::aruco::estimatePoseBoard(m_pimpl->currentDetectedMarkerCorners,
                                                         m_pimpl->currentDetectedMarkerIds,
                                                         m_pimpl->board,
                                                         m_pimpl->cameraMatrix,
                                                         m_pimpl->distCoeff,
                                                         rvec,
                                                         tvec);

        if (valid_corners > 0)
        {
            // Board detected
            // Convert to Eigen matrices
            cv::Rodrigues(rvec, m_pimpl->R);
            cv::cv2eigen(m_pimpl->R, m_pimpl->Reig);
            m_pimpl->teig << tvec[0], tvec[1], tvec[2];

            m_pimpl->poseEig = toEigenPose(m_pimpl->Reig, m_pimpl->teig);

            // Store the board's pose as marker id 0
            ArucoData boardData{0,
                                       {}, // No corners for board
                                       m_pimpl->poseEig};
            m_pimpl->out.markers[0] = boardData;
        }
        else
        {
            log()->debug("{} Board not detected (or not enough markers). Falling back to single marker estimation.", printPrefix);
            m_pimpl->estimateSingleMarkersPose();
        }
    } else {
        // isBoard is false, so estimate single marker poses
        m_pimpl->estimateSingleMarkersPose();
    }

    m_pimpl->out.timeNow = m_pimpl->currentTime;
    return true;
}

void ArucoDetector::Impl::estimateSingleMarkersPose() {
    // Estimate pose of single markers
    cv::aruco::estimatePoseSingleMarkers(currentDetectedMarkerCorners,
                                         markerLength,
                                         cameraMatrix,
                                         distCoeff,
                                         currentDetectedMarkersRotVecs,
                                         currentDetectedMarkersTransVecs);

    for (std::size_t idx = 0; idx < currentDetectedMarkerIds.size(); idx++)
    {
        cv::Rodrigues(currentDetectedMarkersRotVecs[idx], R);
        cv::cv2eigen(R, Reig);
        teig << currentDetectedMarkersTransVecs[idx](0),
            currentDetectedMarkersTransVecs[idx](1),
            currentDetectedMarkersTransVecs[idx](2);

        poseEig = toEigenPose(Reig, teig);
        ArucoData markerData{currentDetectedMarkerIds[idx],
                                   currentDetectedMarkerCorners[idx],
                                   poseEig};
        out.markers[currentDetectedMarkerIds[idx]] = markerData;
    }
}

const ArucoDetectorOutput& ArucoDetector::getOutput() const
{
    return m_pimpl->out;
}

bool ArucoDetector::isOutputValid() const
{
    if (!m_pimpl->initialized)
    {
        return false;
    }

    return true;
}

bool ArucoDetector::getDetectedMarkerData(const int& id, ArucoData& Data)
{
    if (m_pimpl->out.markers.find(id) == m_pimpl->out.markers.end())
    {
        return false;
    }

    Data = m_pimpl->out.markers.at(id);
    return true;
}

bool ArucoDetector::getImageWithDetectedMarkers(cv::Mat& outputImg,
                                                const bool& drawFrames,
                                                const double& axisLengthForDrawing)
{
    if (m_pimpl->currentImg.empty())
    {
        return false;
    }

    m_pimpl->currentImg.copyTo(outputImg);

    if (outputImg.empty())
    {
        return false;
    }

    cv::aruco::drawDetectedMarkers(outputImg,
                                   m_pimpl->currentDetectedMarkerCorners,
                                   m_pimpl->currentDetectedMarkerIds);

    if (drawFrames && m_pimpl->out.markers.find(0) != m_pimpl->out.markers.end()) // Assuming board
                                                                                  // ID is 0
    {
        // Draw coordinate axes on the board
        cv::Vec3d rvec, tvec;

        // Convert Eigen pose back to OpenCV rvec and tvec
        Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
        Eigen::Vector3d translationVector = Eigen::Vector3d::Zero();
        if (m_pimpl->out.markers.count(0))
        {
            rotationMatrix = m_pimpl->out.markers.at(0).pose.block<3, 3>(0, 0);
            translationVector = m_pimpl->out.markers.at(0).pose.block<3, 1>(0, 3);
        }

        cv::eigen2cv(rotationMatrix, m_pimpl->R); // Convert Eigen rotation to OpenCV Mat
        cv::Rodrigues(m_pimpl->R, rvec); // Convert rotation matrix to rotation vector

        tvec[0] = translationVector(0);
        tvec[1] = translationVector(1);
        tvec[2] = translationVector(2);

        cv::drawFrameAxes(outputImg,
                          m_pimpl->cameraMatrix,
                          m_pimpl->distCoeff,
                          rvec, // rotation vector
                          tvec, // translation vector
                          axisLengthForDrawing);
    }

    return true;
}

void ArucoDetector::Impl::resetBuffers()
{
    currentDetectedMarkerCorners.clear();
    currentDetectedMarkerIds.clear();
    currentDetectedMarkersRotVecs.clear();
    currentDetectedMarkersTransVecs.clear();
    out.markers.clear();
    out.timeNow = -1.0;
}