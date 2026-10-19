// Saves one lidar scan and one image of every camera, as the main application sees them
// (images rotated as configured in sensors.json). Used to check the lidar-camera calibration offline.
// Usage: capture_snapshot <output directory>
// Writes <dir>/cloud.ply and <dir>/<camN>.png
#include <Camera/Daheng.h>
#include <Lidar/Ouster.h>
#include <Utils/AsyncWriter.h>
#include <Utils/SensorConfig.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <output directory>\n";
        return 1;
    }
    const std::filesystem::path outDir = argv[1];
    std::filesystem::create_directories(outDir);

    std::vector<std::shared_ptr<Daheng>> cameras;
    for (const CameraInfo& camInfo : sensorConfig().cameras) {
        cameras.push_back(std::make_shared<Daheng>(camInfo.name, camInfo.ip, camInfo.rFlag));
        cameras.back()->start();
    }

    std::unique_ptr<Ouster> ouster;
    try {
        ouster = std::make_unique<Ouster>(sensorConfig().lidarIp);
    } catch (const std::exception& e) {
        std::cerr << "Lidar not reachable at " << sensorConfig().lidarIp << ": " << e.what() << std::endl;
        return 1;
    }

    // Let the cameras settle their exposure
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // A complete scan, then the images right after it
    for (int attempt = 0; attempt < 50 && !ouster->scan(0.2); ++attempt) {}
    if (!ouster->getPointCloud() || !ouster->savePLY((outDir / "cloud.ply").string())) {
        std::cerr << "No lidar scan received\n";
        return 1;
    }
    for (size_t i = 0; i < cameras.size(); ++i) {
        const std::string name = sensorConfig().cameras[i].name;
        std::shared_ptr<cv::Mat> imageBGR = cameras[i]->getCvImage();
        if (!imageBGR || imageBGR->empty()) {
            std::cerr << "No image from " << name << "\n";
            continue;
        }
        cv::imwrite((outDir / (name + ".png")).string(), *imageBGR);
        std::cout << name << ": " << imageBGR->cols << "x" << imageBGR->rows << "\n";
    }

    for (auto& camera : cameras) {
        camera->stop();
    }
    AsyncWriter::instance().flush();
    std::cout << "Saved to " << outDir << "\n";
    return 0;
}
