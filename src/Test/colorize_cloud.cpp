// Colours a recorded point cloud with recorded camera images, as the main application does in RGB mode.
// Usage: colorize_cloud <calib.json> <cloud.ply> <out.ply> camN=<image.png> [camM=<image.png> ...]
// The output is a binary PLY with x, y, z, red, green, blue.
#include <Utils/LidarCamera.h>
#include <Utils/PointCloudColor.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char* argv[])
{
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <calib.json> <cloud.ply> <out.ply> camN=<image.png> [camM=<image.png> ...]\n";
        return 1;
    }
    const std::string calibFile = argv[1];

    pcl::PointCloud<pcl::PointXYZ> input;
    if (pcl::io::loadPLYFile(argv[2], input) != 0) {
        std::cerr << "Cannot read " << argv[2] << "\n";
        return 1;
    }
    std::vector<vaiop::QtPoint> cloud;
    cloud.reserve(input.size());
    for (const pcl::PointXYZ& point : input) {
        cloud.push_back({QVector3D(point.x, point.y, point.z), QVector3D()});
    }

    std::vector<CameraCalib> calibs;
    std::vector<cv::Mat> imagesRGB;
    for (int i = 4; i < argc; ++i) {
        const std::string arg = argv[i];
        const size_t eq = arg.find('=');
        if (eq == std::string::npos) {
            std::cerr << "Expected camN=<image>, got " << arg << "\n";
            return 1;
        }
        const std::string cam = arg.substr(0, eq);
        CameraCalib calib = loadCameraCalib(calibFile, cam);
        cv::Mat image = cv::imread(arg.substr(eq + 1), cv::IMREAD_COLOR);
        if (!calib.valid || image.empty()) {
            std::cerr << "Skipping " << cam << ": " << (calib.valid ? "cannot read the image" : "no calibration") << "\n";
            continue;
        }
        if (calib.imageSize.area() > 0 && image.size() != calib.imageSize) {
            std::cerr << "Warning: " << cam << " image is " << image.cols << "x" << image.rows << ", calibrated for "
                      << calib.imageSize.width << "x" << calib.imageSize.height << ": it will be skipped\n";
        }
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        calibs.push_back(std::move(calib));
        imagesRGB.push_back(std::move(image));
    }

    const CameraColorizer colorizer(std::move(calibs));
    const size_t nColored = colorizer.colorize(cloud, imagesRGB);
    std::cout << nColored << " of " << cloud.size() << " points coloured by " << imagesRGB.size() << " camera(s)\n";

    pcl::PointCloud<pcl::PointXYZRGB> output;
    output.reserve(cloud.size());
    for (const vaiop::QtPoint& p : cloud) {
        pcl::PointXYZRGB q(static_cast<uint8_t>(p.color.x() * 255.0f + 0.5f),
                           static_cast<uint8_t>(p.color.y() * 255.0f + 0.5f),
                           static_cast<uint8_t>(p.color.z() * 255.0f + 0.5f));
        q.x = p.position.x();
        q.y = p.position.y();
        q.z = p.position.z();
        output.push_back(q);
    }
    if (pcl::io::savePLYFileBinary(argv[3], output) != 0) {
        std::cerr << "Cannot write " << argv[3] << "\n";
        return 1;
    }
    return 0;
}
