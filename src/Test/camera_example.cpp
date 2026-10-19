
#include <Camera/Daheng.h>
#include <Utils/SensorConfig.h>
#include <memory>
#include <iostream>
#include <vector>

// Usage: camera_example [calibration file]
int main(int argc, char const *argv[])
{
    // 0. Parse arguments
    std::string calibFileName = "";
    if (argc > 1) { // argc > 1 means there is at least one argument after the program name
        calibFileName = std::string(argv[1]);
    }

    // 1. Open devices (same addresses and rotations as the main application)
    std::vector<std::shared_ptr<Daheng>> cameras;
    for (const CameraInfo& camInfo : sensorConfig().cameras) {
        cameras.push_back(std::make_shared<Daheng>(camInfo.name, camInfo.ip, camInfo.rFlag, calibFileName));
    }

    for (auto& camPtr : cameras) {
        camPtr->start();
    }

    // 2. Initialize viewer
    cv::namedWindow("Image Viewer", cv::WINDOW_NORMAL);
    // rotated image size is (height, width) = (804, 552)
    const int imgWidth = 552;
    const int imgHeight = 804;
    std::vector<cv::Mat> images;
    for (size_t i = 0; i < cameras.size(); i++) {
        images.push_back(cv::Mat::zeros(imgHeight, imgWidth, CV_8UC3));
    }

    while (true)
    {
        for (size_t i = 0; i < cameras.size(); i++) {
            std::shared_ptr<cv::Mat> imgPtr = cameras[i]->getCvImage();
            if (imgPtr != nullptr && !imgPtr->empty()) {
                // hconcat needs the same height for every image
                cv::resize(*imgPtr, images[i], cv::Size(imgWidth, imgHeight));
            }
        }

        // Concatenate horizontally
        cv::Mat imgViz;
        cv::hconcat(images, imgViz);
        cv::imshow("Image Viewer", imgViz);

        // Wait for 1ms; check if 'q' was pressed
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 'q' || key == 'Q') {
            break;
        }
    }

    for (auto& camPtr : cameras) {
        camPtr->stop();
    }

    return 0;
}
