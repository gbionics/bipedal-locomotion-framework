/**
 * @file ArucoDetectorTest.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <opencv2/opencv.hpp>

#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <ResourceFolderPath.h>

#include <iostream>
#include <chrono>
#include <thread>

using namespace BipedalLocomotion::Perception;
using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("Aruco Detector - Live Camera Feed with Boards")
{
    std::shared_ptr<IParametersHandler> parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("is_board", true);
    parameterHandler->setParameter("marker_dictionary", "6X6_250");
    parameterHandler->setParameter("marker_length", 0.1); // Reduced marker length to test with small markers
    parameterHandler->setParameter("markers_x", 5); // Number of markers in X direction
    parameterHandler->setParameter("markers_y", 7); // Number of markers in Y direction
    parameterHandler->setParameter("marker_separation", 0.02); // Marker separation in meters
    parameterHandler->setParameter("camera_matrix", std::vector<double>{600, 0, 320, 0, 600, 240, 0, 0, 1}); // Example Values, replace with calibrated ones
    parameterHandler->setParameter("distortion_coefficients", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
    std::cout << "Parameters set" << std::endl;

    // Instantiate the detector
    ArucoDetector detector;
    REQUIRE(detector.initialize(parameterHandler));

    // Initialize video capture
    cv::VideoCapture cap(0); // Open the default camera
    REQUIRE(cap.isOpened());

    cv::namedWindow("Aruco Detection - Live with Boards", cv::WINDOW_AUTOSIZE);

    auto startTime = std::chrono::steady_clock::now();
    auto endTime = startTime + std::chrono::seconds(20); // Run for 10 seconds

    while (std::chrono::steady_clock::now() < endTime)
    {
        cv::Mat frame;
        cap >> frame;
        REQUIRE(!frame.empty());

        REQUIRE(detector.setImage(frame, 0.0)); // Time is not important for this test
        REQUIRE(detector.advance());

        cv::Mat outputImage;
        if (!detector.getImageWithDetectedMarkers(outputImage, true, 0.05)) // Draw frames and reduce axisLength
        {
            outputImage = frame.clone();
        }

        // Check for board detection
        ArucoData boardData;
        bool boardDetected = detector.getDetectedMarkerData(0, boardData); // Check for board with id 0
        if (boardDetected)
        {
            std::cout << "Board Detected! Pose:" << std::endl << boardData.pose << std::endl;
        }
        else
        {
            std::cout << "No Board Detected." << std::endl;
        }

        cv::imshow("Aruco Detection - Live with Boards", outputImage);

        // Small delay to allow the GUI to update and process key presses
        if (cv::waitKey(1) == 'q')
        {
            break; // exit if q is pressed
        }
    }

    // Cleanup
    cap.release();
    cv::destroyAllWindows();
}