/**
 * @file ArucoDetector.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/CommonConversions.h>
#include <BipedalLocomotion/GenericContainer/Vector.h>
#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::GenericContainer;
using namespace BipedalLocomotion::Perception;
using namespace BipedalLocomotion::Conversions;

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#else
#include <opencv2/aruco.hpp>
#endif
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

class ArucoDetector::Impl
{
public:
    /**
     * clear all internal buffers
     */
    void resetBuffers();

    // parameters
    cv::Ptr<cv::aruco::Dictionary> dictionary; /**< container with detected markers data */
#if CV_VERSION_MAJOR >= 5
    std::unique_ptr<cv::aruco::ArucoDetector> arucoDetector;
#endif
    double markerLength; /**< marker length*/
    cv::Mat cameraMatrix; /**< camera calibration matrix*/
    cv::Mat distCoeff; /**< camera distortion coefficients*/

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
// instead of a cv::Ptr<cv::aruco::Dictionary>
#if (CV_VERSION_MAJOR >= 5) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    m_pimpl->dictionary = cv::makePtr<cv::aruco::Dictionary>();
    *(m_pimpl->dictionary) = cv::aruco::getPredefinedDictionary(m_pimpl->availableDict.at(dictName));
#if CV_VERSION_MAJOR >= 5
    m_pimpl->arucoDetector
        = std::make_unique<cv::aruco::ArucoDetector>(*(m_pimpl->dictionary));
#endif
#else
    m_pimpl->dictionary = cv::aruco::getPredefinedDictionary(m_pimpl->availableDict.at(dictName));
#endif

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
#if CV_VERSION_MAJOR >= 5
    m_pimpl->arucoDetector->detectMarkers(m_pimpl->currentImg,
                                          m_pimpl->currentDetectedMarkerCorners,
                                          m_pimpl->currentDetectedMarkerIds);
#else
    cv::aruco::detectMarkers(m_pimpl->currentImg,
                             m_pimpl->dictionary,
                             m_pimpl->currentDetectedMarkerCorners,
                             m_pimpl->currentDetectedMarkerIds);
#endif

    if (m_pimpl->currentDetectedMarkerIds.size() > 0)
    {
#if CV_VERSION_MAJOR >= 5
        const float halfLength = static_cast<float>(m_pimpl->markerLength / 2.0);
        const std::vector<cv::Point3f> markerPoints{{-halfLength, halfLength, 0.0F},
                                                    {halfLength, halfLength, 0.0F},
                                                    {halfLength, -halfLength, 0.0F},
                                                    {-halfLength, -halfLength, 0.0F}};

        m_pimpl->currentDetectedMarkersRotVecs.resize(
            m_pimpl->currentDetectedMarkerCorners.size());
        m_pimpl->currentDetectedMarkersTransVecs.resize(
            m_pimpl->currentDetectedMarkerCorners.size());

        for (std::size_t idx = 0; idx < m_pimpl->currentDetectedMarkerCorners.size(); idx++)
        {
            cv::solvePnP(markerPoints,
                         m_pimpl->currentDetectedMarkerCorners[idx],
                         m_pimpl->cameraMatrix,
                         m_pimpl->distCoeff,
                         m_pimpl->currentDetectedMarkersRotVecs[idx],
                         m_pimpl->currentDetectedMarkersTransVecs[idx],
                         false,
                         cv::SOLVEPNP_ITERATIVE);
        }
#else
        cv::aruco::estimatePoseSingleMarkers(m_pimpl->currentDetectedMarkerCorners,
                                             m_pimpl->markerLength,
                                             m_pimpl->cameraMatrix,
                                             m_pimpl->distCoeff,
                                             m_pimpl->currentDetectedMarkersRotVecs,
                                             m_pimpl->currentDetectedMarkersTransVecs);
#endif

        for (std::size_t idx = 0; idx < m_pimpl->currentDetectedMarkerIds.size(); idx++)
        {
            cv::Rodrigues(m_pimpl->currentDetectedMarkersRotVecs[idx], m_pimpl->R);
            cv::cv2eigen(m_pimpl->R, m_pimpl->Reig);
            m_pimpl->teig << m_pimpl->currentDetectedMarkersTransVecs[idx](0),
                m_pimpl->currentDetectedMarkersTransVecs[idx](1),
                m_pimpl->currentDetectedMarkersTransVecs[idx](2);

            m_pimpl->poseEig = toEigenPose(m_pimpl->Reig, m_pimpl->teig);
            ArucoMarkerData markerData{m_pimpl->currentDetectedMarkerIds[idx],
                                       m_pimpl->currentDetectedMarkerCorners[idx],
                                       m_pimpl->poseEig};
            m_pimpl->out.markers[m_pimpl->currentDetectedMarkerIds[idx]] = markerData;
        }
        m_pimpl->out.timeNow = m_pimpl->currentTime;
    }

    return true;
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

bool ArucoDetector::getDetectedMarkerData(const int& id, ArucoMarkerData& markerData)
{
    if (m_pimpl->out.markers.find(id) == m_pimpl->out.markers.end())
    {
        return false;
    }

    markerData = m_pimpl->out.markers.at(id);
    return true;
}

bool ArucoDetector::getImageWithDetectedMarkers(cv::Mat& outputImg,
                                                const bool& drawFrames,
                                                const double& axisLengthForDrawing)
{
    std::size_t nrDetectedMarkers = m_pimpl->currentDetectedMarkerIds.size();
    if (m_pimpl->currentImg.empty() || nrDetectedMarkers <= 0)
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

    if (drawFrames)
    {
        for (std::size_t idx = 0; idx < nrDetectedMarkers; idx++)
        {
            cv::drawFrameAxes(outputImg,
                              m_pimpl->cameraMatrix,
                              m_pimpl->distCoeff,
                              m_pimpl->currentDetectedMarkersRotVecs[idx],
                              m_pimpl->currentDetectedMarkersTransVecs[idx],
                              axisLengthForDrawing);
        }
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
