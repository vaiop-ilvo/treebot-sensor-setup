#include <Utils/GroundPlane.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace {
    // Candidate points: within this range of the lidar [m] (the ground is sparse and noisy far away)
    constexpr float MIN_RANGE = 1.0f;
    constexpr float MAX_RANGE = 25.0f;
    // At most this many candidates are used for RANSAC
    constexpr size_t MAX_CANDIDATES = 6000;
    constexpr int RANSAC_ITERATIONS = 200;
    // Distance of an inlier to the plane [m]
    constexpr float INLIER_DISTANCE = 0.08f;
    // Largest angle between the ground normal and the lidar z axis (the lidar may be mounted tilted)
    constexpr float MAX_TILT_DEG = 35.0f;
    // The lidar is at least this high above the ground [m]
    constexpr float MIN_SENSOR_HEIGHT = 0.2f;
    // Minimum number of inliers for a valid plane
    constexpr size_t MIN_INLIERS = 200;
    // Temporal smoothing: a new plane close to the previous one is blended in with this weight
    constexpr float SMOOTHING = 0.3f;
    constexpr float SMOOTH_MAX_ANGLE_DEG = 5.0f;
    constexpr float SMOOTH_MAX_OFFSET = 0.15f;

    constexpr float DEG = 3.14159265f / 180.0f;

    // Orients the plane so the normal points towards the sensor (d > 0); false when it is not a ground candidate
    bool asGround(Eigen::Vector3f n, float d, GroundPlane& plane)
    {
        const float length = n.norm();
        if (length < 1e-6f) {
            return false;
        }
        n /= length;
        d /= length;
        if (d < 0.0f) {
            n = -n;
            d = -d;
        }
        // Normal towards the sensor must point up: a ceiling (sensor below it) has a downward normal
        if (n.z() < std::cos(MAX_TILT_DEG * DEG) || d < MIN_SENSOR_HEIGHT) {
            return false;
        }
        plane.normal = n;
        plane.d = d;
        return true;
    }
}

float GroundPlane::tiltDeg() const
{
    return std::acos(std::clamp(normal.z(), -1.0f, 1.0f)) / DEG;
}

GroundPlane fitGroundPlane(const std::vector<vaiop::QtPoint>& cloud, const GroundPlane* previous)
{
    // 1. Candidates: evenly subsampled points in range
    std::vector<Eigen::Vector3f> candidates;
    const size_t stride = std::max<size_t>(1, cloud.size() / (MAX_CANDIDATES * 2));
    for (size_t i = 0; i < cloud.size() && candidates.size() < MAX_CANDIDATES; i += stride) {
        const Eigen::Vector3f p(cloud[i].position.x(), cloud[i].position.y(), cloud[i].position.z());
        const float range = p.norm();
        if (range >= MIN_RANGE && range <= MAX_RANGE) {
            candidates.push_back(p);
        }
    }
    GroundPlane best;
    if (candidates.size() < MIN_INLIERS) {
        return best;
    }

    auto countInliers = [&candidates](const GroundPlane& plane) {
        size_t n = 0;
        for (const Eigen::Vector3f& p : candidates) {
            n += std::abs(plane.height(p)) < INLIER_DISTANCE ? 1 : 0;
        }
        return n;
    };

    // 2. RANSAC over planes through 3 random candidates; the previous plane competes as well
    std::mt19937 rng(12345);
    std::uniform_int_distribution<size_t> pick(0, candidates.size() - 1);
    if (previous && previous->valid) {
        best = *previous;
        best.inliers = countInliers(best);
    }
    for (int it = 0; it < RANSAC_ITERATIONS; ++it) {
        const Eigen::Vector3f& a = candidates[pick(rng)];
        const Eigen::Vector3f& b = candidates[pick(rng)];
        const Eigen::Vector3f& c = candidates[pick(rng)];
        const Eigen::Vector3f n = (b - a).cross(c - a);
        GroundPlane plane;
        if (!asGround(n, -n.dot(a), plane)) {
            continue;
        }
        plane.inliers = countInliers(plane);
        if (plane.inliers > best.inliers) {
            best = plane;
        }
    }
    if (best.inliers < MIN_INLIERS) {
        return GroundPlane{};
    }

    // 3. Least squares on the inliers: the normal is the direction of least variance
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    std::vector<Eigen::Vector3f> inliers;
    for (const Eigen::Vector3f& p : candidates) {
        if (std::abs(best.height(p)) < INLIER_DISTANCE) {
            inliers.push_back(p);
            mean += p;
        }
    }
    mean /= static_cast<float>(inliers.size());
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const Eigen::Vector3f& p : inliers) {
        covariance += (p - mean) * (p - mean).transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    GroundPlane refined;
    if (asGround(solver.eigenvectors().col(0), -solver.eigenvectors().col(0).dot(mean), refined)) {
        refined.inliers = countInliers(refined);
        if (refined.inliers >= best.inliers) {
            best = refined;
        }
    }
    best.valid = true;

    // 4. Smoothing over time, so the view does not jitter; a real change of the ground is followed immediately
    if (previous && previous->valid) {
        const float angle = std::acos(std::clamp(best.normal.dot(previous->normal), -1.0f, 1.0f)) / DEG;
        if (angle < SMOOTH_MAX_ANGLE_DEG && std::abs(best.d - previous->d) < SMOOTH_MAX_OFFSET) {
            Eigen::Vector3f n = (1.0f - SMOOTHING) * previous->normal + SMOOTHING * best.normal;
            const float d = (1.0f - SMOOTHING) * previous->d + SMOOTHING * best.d;
            const float length = n.norm();
            best.normal = n / length;
            best.d = d / length;
        }
    }
    return best;
}

namespace {
    // Ground axes: forward = lidar x projected on the plane, left = normal x forward
    void groundAxes(const GroundPlane& plane, Eigen::Vector3f& forward, Eigen::Vector3f& left)
    {
        const Eigen::Vector3f& n = plane.normal;
        forward = Eigen::Vector3f::UnitX() - n.x() * n;
        if (forward.norm() < 1e-3f) {
            forward = Eigen::Vector3f::UnitY() - n.y() * n;
        }
        forward.normalize();
        left = n.cross(forward);
    }
}

std::vector<BirdEyePoint> projectToGround(const std::vector<vaiop::QtPoint>& cloud, const GroundPlane& plane)
{
    std::vector<BirdEyePoint> points;
    if (!plane.valid) {
        return points;
    }
    Eigen::Vector3f forward, left;
    groundAxes(plane, forward, left);

    points.reserve(cloud.size());
    for (const vaiop::QtPoint& q : cloud) {
        const Eigen::Vector3f p(q.position.x(), q.position.y(), q.position.z());
        points.push_back({forward.dot(p), left.dot(p), plane.height(p), q.color});
    }
    return points;
}

std::vector<BirdEyeBlob> projectToGround(const std::vector<Blob>& blobs, const GroundPlane& plane)
{
    std::vector<BirdEyeBlob> result;
    if (!plane.valid) {
        return result;
    }
    Eigen::Vector3f forward, left;
    groundAxes(plane, forward, left);

    result.reserve(blobs.size());
    for (const Blob& blob : blobs) {
        const Eigen::Vector3f p(blob.center.x(), blob.center.y(), blob.center.z());
        result.push_back({forward.dot(p), left.dot(p), plane.height(p), blob.radius, blob.color});
    }
    return result;
}

cv::Mat renderBirdEye(const std::vector<BirdEyePoint>& points, const BirdEyeView& view, const cv::Vec3b& bg)
{
    cv::Mat image(view.size, CV_8UC3, cv::Scalar(bg[0], bg[1], bg[2]));
    cv::Mat top(view.size, CV_32F, cv::Scalar(-std::numeric_limits<float>::infinity()));
    const int half = view.pointSize / 2;

    for (const BirdEyePoint& p : points) {
        if (p.height < view.minHeight || p.height > view.maxHeight) {
            continue;
        }
        const cv::Point2f px = view.toPixel(p.forward, p.left);
        const int u0 = static_cast<int>(px.x) - half;
        const int v0 = static_cast<int>(px.y) - half;
        if (u0 + view.pointSize <= 0 || v0 + view.pointSize <= 0 || u0 >= view.size.width || v0 >= view.size.height) {
            continue;
        }
        // Ground dimmer, so the objects on it stand out
        const float gain = p.height <= view.groundHeight ? 0.55f : 1.0f;
        const cv::Vec3b color(static_cast<uint8_t>(std::clamp(p.color.x() * gain, 0.0f, 1.0f) * 255.0f),
                              static_cast<uint8_t>(std::clamp(p.color.y() * gain, 0.0f, 1.0f) * 255.0f),
                              static_cast<uint8_t>(std::clamp(p.color.z() * gain, 0.0f, 1.0f) * 255.0f));
        for (int v = std::max(0, v0); v < std::min(view.size.height, v0 + view.pointSize); ++v) {
            float* topRow = top.ptr<float>(v);
            cv::Vec3b* row = image.ptr<cv::Vec3b>(v);
            for (int u = std::max(0, u0); u < std::min(view.size.width, u0 + view.pointSize); ++u) {
                if (p.height > topRow[u]) {
                    topRow[u] = p.height;
                    row[u] = color;
                }
            }
        }
    }
    return image;
}
