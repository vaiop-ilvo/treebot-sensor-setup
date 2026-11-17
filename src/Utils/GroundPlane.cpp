#include <Utils/GroundPlane.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace {
    // Candidate points: within this range of the lidar [m] (the ground is sparse and noisy far away)
    constexpr float MIN_RANGE = 1.0f;
    constexpr float MAX_RANGE = 25.0f;
    // At most this many candidates are used for the search
    constexpr size_t MAX_CANDIDATES = 6000;
    // Distance of an inlier to the plane [m]
    constexpr float INLIER_DISTANCE = 0.08f;
    // Points this far below a plane count against it: almost nothing is below the real ground (only noise and
    // reflections), while a slanted plane through walls or machines has many of their points below it
    constexpr float BELOW_DISTANCE = 0.25f;
    constexpr int BELOW_PENALTY = 2;
    // Largest angle between the ground normal and the lidar z axis (the lidar may be mounted tilted)
    constexpr float MAX_TILT_DEG = 35.0f;
    // Height range of the lidar above the ground [m]
    constexpr float MIN_SENSOR_HEIGHT = 0.2f;
    constexpr float MAX_SENSOR_HEIGHT = 2.5f;
    // Tilt search: a coarse grid over all tilts, then a fine grid around the best one [deg]
    constexpr float COARSE_STEP_DEG = 2.5f;
    constexpr float FINE_STEP_DEG = 0.5f;
    // Height histogram per tilt [m]
    constexpr float BIN = 0.02f;
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

    // Ground normal for a tilt about the lidar x axis (roll) and y axis (pitch) [rad]
    Eigen::Vector3f tiltedNormal(float roll, float pitch)
    {
        return Eigen::Vector3f(std::sin(pitch), -std::sin(roll) * std::cos(pitch), std::cos(roll) * std::cos(pitch));
    }

    struct Candidate {
        GroundPlane plane;
        long score = std::numeric_limits<long>::min();
        float roll = 0.0f;
        float pitch = 0.0f;
    };

    // Best ground height for one normal: a histogram of the point heights along the normal; the ground is the
    // height with the most points within INLIER_DISTANCE, minus the penalty for the points below it
    Candidate bestHeight(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& normal, float roll, float pitch)
    {
        // Heights h = n.p; the ground is at h = -d, d in [MIN_SENSOR_HEIGHT, MAX_SENSOR_HEIGHT]
        constexpr float LOW = -MAX_SENSOR_HEIGHT - BELOW_DISTANCE - INLIER_DISTANCE;
        constexpr float HIGH = -MIN_SENSOR_HEIGHT + INLIER_DISTANCE;
        constexpr int BINS = static_cast<int>((HIGH - LOW) / BIN) + 1;
        std::array<int, BINS> histogram{};
        int belowAll = 0;           // Below the histogram
        for (const Eigen::Vector3f& p : points) {
            const float h = normal.dot(p);
            if (h < LOW) {
                belowAll++;
            } else if (h < HIGH) {
                histogram[static_cast<int>((h - LOW) / BIN)]++;
            }
        }
        std::array<int, BINS + 1> cumulative{};     // Points below the start of each bin
        cumulative[0] = belowAll;
        for (int i = 0; i < BINS; ++i) {
            cumulative[i + 1] = cumulative[i] + histogram[i];
        }
        constexpr int HALF = static_cast<int>(INLIER_DISTANCE / BIN);
        constexpr int BELOW_BINS = static_cast<int>(BELOW_DISTANCE / BIN);
        Candidate best;
        for (int center = HALF + BELOW_BINS; center + HALF < BINS; ++center) {
            const float d = -(LOW + (center + 0.5f) * BIN);
            if (d < MIN_SENSOR_HEIGHT || d > MAX_SENSOR_HEIGHT) {
                continue;
            }
            const long inliers = cumulative[center + HALF + 1] - cumulative[center - HALF];
            const long below = cumulative[center - BELOW_BINS];
            const long score = inliers - BELOW_PENALTY * below;
            if (score > best.score) {
                best.score = score;
                best.plane.normal = normal;
                best.plane.d = d;
                best.plane.inliers = static_cast<size_t>(inliers);
                best.roll = roll;
                best.pitch = pitch;
            }
        }
        return best;
    }

    // The best plane over a grid of tilts around (roll0, pitch0) [deg]
    Candidate searchTilts(const std::vector<Eigen::Vector3f>& points, float roll0, float pitch0, float halfRange, float step)
    {
        Candidate best;
        for (float roll = roll0 - halfRange; roll <= roll0 + halfRange + 1e-3f; roll += step) {
            for (float pitch = pitch0 - halfRange; pitch <= pitch0 + halfRange + 1e-3f; pitch += step) {
                const Eigen::Vector3f n = tiltedNormal(roll * DEG, pitch * DEG);
                if (n.z() < std::cos(MAX_TILT_DEG * DEG)) {
                    continue;
                }
                const Candidate candidate = bestHeight(points, n, roll, pitch);
                if (candidate.score > best.score) {
                    best = candidate;
                }
            }
        }
        return best;
    }
}

float GroundPlane::tiltDeg() const
{
    return std::acos(std::clamp(normal.z(), -1.0f, 1.0f)) / DEG;
}

GroundPlane fitGroundPlane(const std::vector<vaiop::QtPoint>& cloud, const GroundPlane* previous)
{
    // 1. Candidates: points in range, evenly subsampled over the whole scan. (The Ouster cloud is ordered by beam,
    //    upper beams first: stopping early would leave out the lower beams, which see most of the ground.)
    std::vector<Eigen::Vector3f> candidates;
    candidates.reserve(MAX_CANDIDATES + 1);
    const size_t stride = std::max<size_t>(1, (cloud.size() + MAX_CANDIDATES - 1) / MAX_CANDIDATES);
    for (size_t i = 0; i < cloud.size(); i += stride) {
        const Eigen::Vector3f p(cloud[i].position.x(), cloud[i].position.y(), cloud[i].position.z());
        const float range = p.norm();
        if (range >= MIN_RANGE && range <= MAX_RANGE) {
            candidates.push_back(p);
        }
    }
    if (candidates.size() < MIN_INLIERS) {
        return GroundPlane{};
    }
    auto countInliers = [&candidates](const GroundPlane& plane) {
        size_t n = 0;
        for (const Eigen::Vector3f& p : candidates) {
            n += std::abs(plane.height(p)) < INLIER_DISTANCE ? 1 : 0;
        }
        return n;
    };

    // 2. Systematic search over all tilts (deterministic, unlike random planes through 3 points: in a hall only a
    //    small part of the points is floor, and a random search easily settles on a slanted plane through the walls)
    const Candidate coarse = searchTilts(candidates, 0.0f, 0.0f, MAX_TILT_DEG, COARSE_STEP_DEG);
    if (coarse.score < static_cast<long>(MIN_INLIERS)) {
        return GroundPlane{};
    }
    const Candidate fine = searchTilts(candidates, coarse.roll, coarse.pitch, COARSE_STEP_DEG, FINE_STEP_DEG);
    GroundPlane best = fine.score >= coarse.score ? fine.plane : coarse.plane;
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

std::vector<BirdEyePoint> projectToGround(const std::vector<vaiop::QtPoint>& cloud, const GroundPlane& plane,
                                          const std::vector<uint8_t>* labels)
{
    std::vector<BirdEyePoint> points;
    if (!plane.valid) {
        return points;
    }
    Eigen::Vector3f forward, left;
    groundAxes(plane, forward, left);

    const bool labelled = labels && labels->size() == cloud.size();
    points.reserve(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
        const vaiop::QtPoint& q = cloud[i];
        const Eigen::Vector3f p(q.position.x(), q.position.y(), q.position.z());
        points.push_back({forward.dot(p), left.dot(p), plane.height(p), q.color,
                          labelled ? (*labels)[i] : BirdEyePoint::NO_LABEL});
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
