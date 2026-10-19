// Colours a recorded point cloud with recorded camera images, as the main application does in RGB mode,
// or with the class colours of a segmentation model (segmentation mode) when --model is given.
// Usage: colorize_cloud <calib.json> <cloud.ply> <out.ply> camN=<image.png> [camM=<image.png> ...] [--model <model dir>] [--bev <out.png>] [--blobs <min points>]
// The output is a binary PLY with x, y, z, red, green, blue. --bev also fits the ground plane and writes the
// coloured cloud projected on it (bird's-eye view, forward up, rings every 5 m). With --model, --bev and --blobs, the
// bird's-eye view shows only the blobs of at least <min points> points, with their circles (as the Blobs button).
#include <Utils/Blobs.h>
#include <Utils/GroundPlane.h>
#include <Utils/LidarCamera.h>
#include <Utils/PointCloudColor.h>
#include <Utils/Segmenter.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char* argv[])
{
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <calib.json> <cloud.ply> <out.ply> camN=<image.png> [camM=<image.png> ...] [--model <model dir>] [--bev <out.png>] [--blobs <min points>]\n";
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
    SegmentationConfig segConfig;
    std::string bevFile;
    uint32_t minBlobPoints = 0;
    for (int i = 4; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--model" && i + 1 < argc) {
            segConfig.modelDir = argv[++i];
            continue;
        }
        if (arg == "--blobs" && i + 1 < argc) {
            minBlobPoints = static_cast<uint32_t>(std::stoul(argv[++i]));
            continue;
        }
        if (arg == "--bev" && i + 1 < argc) {
            bevFile = argv[++i];
            continue;
        }
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
    size_t nColored = 0;
    std::vector<uint8_t> labels;
    std::vector<cv::Vec3b> palette;
    const auto tic = std::chrono::steady_clock::now();
    if (segConfig.modelDir.empty()) {
        nColored = colorizer.colorize(cloud, imagesRGB);
    } else {
        Segmenter segmenter(segConfig);
        if (!segmenter.isAvailable()) {
            std::cerr << segmenter.error() << "\n";
            return 1;
        }
        std::vector<cv::Mat> labelMaps;
        for (const cv::Mat& image : imagesRGB) {
            labelMaps.push_back(segmenter.segment(image));
        }
        palette = segmenter.palette();
        nColored = colorizer.colorizeLabels(cloud, labelMaps, palette, &labels);

        std::vector<size_t> counts(segmenter.classNames().size(), 0);
        for (const uint8_t label : labels) {
            if (label < counts.size()) counts[label]++;
        }
        std::cout << "Point classes:";
        for (size_t k = 0; k < counts.size(); ++k) {
            if (counts[k] * 100 >= nColored) {
                std::cout << " " << segmenter.classNames()[k] << " " << counts[k];
            }
        }
        std::cout << "\n";
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tic).count();
    std::cout << nColored << " of " << cloud.size() << " points coloured by " << imagesRGB.size() << " camera(s) in " << ms << " ms\n";

    if (!bevFile.empty()) {
        const auto t0 = std::chrono::steady_clock::now();
        const GroundPlane plane = fitGroundPlane(cloud);
        const double fitMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        if (!plane.valid) {
            std::cerr << "No ground plane found\n";
            return 1;
        }
        std::cout << "Ground: normal (" << plane.normal.transpose() << "), lidar " << plane.sensorHeight() << " m above the ground, tilt "
                  << plane.tiltDeg() << " deg, " << plane.inliers << " inliers, " << fitMs << " ms\n";
        BirdEyeView view;
        view.size = cv::Size(900, 900);
        view.metersPerPixel = 0.035f;
        // Blob view: only the points of the blobs that are large enough (ground left out, as in the application)
        std::vector<vaiop::QtPoint> shown = cloud;
        std::vector<Blob> blobs;
        if (minBlobPoints > 0 && !labels.empty()) {
            std::vector<uint8_t> include(cloud.size());
            for (size_t i = 0; i < cloud.size(); ++i) {
                const QVector3D& p = cloud[i].position;
                include[i] = plane.height(Eigen::Vector3f(p.x(), p.y(), p.z())) > 0.15f ? 1 : 0;
            }
            const auto t1 = std::chrono::steady_clock::now();
            const BlobResult result = findBlobs(cloud, labels, palette, minBlobPoints, 0.3f, include);
            std::cout << result.blobs.size() << " blobs of at least " << minBlobPoints << " points in "
                      << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count() << " ms\n";
            shown.clear();
            for (size_t i = 0; i < cloud.size(); ++i) {
                if (result.pointBlob[i] >= 0) {
                    shown.push_back(cloud[i]);
                }
            }
            blobs = result.blobs;
        }
        cv::Mat bev = renderBirdEye(projectToGround(shown, plane), view, cv::Vec3b(17, 20, 25));
        for (const BirdEyeBlob& blob : projectToGround(blobs, plane)) {
            if (blob.height < view.minHeight || blob.height > view.maxHeight) {
                continue;
            }
            const cv::Scalar color(blob.color.x() * 255, blob.color.y() * 255, blob.color.z() * 255);
            cv::circle(bev, view.toPixel(blob.forward, blob.left), std::max(1, static_cast<int>(blob.radius / view.metersPerPixel)),
                       color, 2, cv::LINE_AA);
        }
        for (int r = 5; r <= 30; r += 5) {
            cv::circle(bev, view.toPixel(0, 0), static_cast<int>(r / view.metersPerPixel), cv::Scalar(70, 75, 80), 1, cv::LINE_AA);
        }
        cv::circle(bev, view.toPixel(0, 0), 5, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        cv::cvtColor(bev, bev, cv::COLOR_RGB2BGR);
        cv::imwrite(bevFile, bev);
    }

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
