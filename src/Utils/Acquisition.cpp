#include <Utils/Acquisition.h>

#include <QDebug>
#include <QString>

#include <algorithm>
#include <cstdlib>

namespace {
    // Maximum time to wait for a camera frame after the trigger (a frame normally arrives within one camera period)
    constexpr auto MAX_CAMERA_WAIT = std::chrono::milliseconds(150);
    // A camera frame further than this from the trigger timestamp is not used (half the 10 Hz period)
    constexpr uint64_t MAX_SYNC_OFFSET_US = 50000;
    // Trigger period when no lidar is connected (preview + recording continue at 10 Hz)
    constexpr auto SOFT_TRIGGER_PERIOD = std::chrono::milliseconds(100);
    // Longest wait for a lidar scan in one loop, so the camera rows complete in time
    constexpr double LIDAR_WAIT_SEC = 0.005;

    uint64_t hostTimeMicros()
    {
        return std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }
}

Acquisition::Acquisition(const SensorConfig& config, std::vector<CameraCalib> calibs, std::function<bool()> isRecording,
                         std::function<void(std::shared_ptr<const Frame>)> onFrame, std::function<void(Connection)> onConnected)
    : config(config),
      calibs(std::move(calibs)),
      isRecording(std::move(isRecording)),
      onFrame(std::move(onFrame)),
      onConnected(std::move(onConnected))
{
}

Acquisition::~Acquisition()
{
    stop();
}

void Acquisition::start()
{
    if (running.exchange(true)) {
        return;
    }
    thread = std::thread(&Acquisition::run, this);
}

void Acquisition::stop()
{
    running = false;
    if (thread.joinable()) {
        thread.join();
    }
    if (robot) {
        robot->stopPolling();
    }
    std::lock_guard<std::mutex> lock(camerasMutex);
    for (auto& camera : cameras) {
        camera->stop();
    }
}

void Acquisition::setPaused(bool isPaused)
{
    paused = isPaused;
}

int Acquisition::rotateCamera(size_t index)
{
    std::lock_guard<std::mutex> lock(camerasMutex);
    if (index >= cameras.size()) {
        return 0;
    }
    cameras[index]->setRotation(cameras[index]->getRotation() + 90);
    return cameras[index]->getRotation();
}

void Acquisition::connect()
{
    Connection connection;

    // Robot (polled in the background, never blocks)
    robot = std::make_unique<ArtofRobot>(config.robotHost, config.robotPort, config.robotKeys);
    robot->startPolling();

    // Cameras
    for (const CameraInfo& camInfo : config.cameras) {
        if (!running) {
            return;
        }
        auto camera = std::make_shared<Daheng>(camInfo.name, camInfo.ip, camInfo.rFlag, camInfo.calibFileName);
        if (camera->isOpen()) {
            camera->start();
            connection.cameraErrors.emplace_back();
        } else {
            connection.cameraErrors.push_back("Cannot open camera at " + camInfo.ip);
        }
        connection.cameraRotations.push_back(camera->getRotation());
        std::lock_guard<std::mutex> lock(camerasMutex);
        cameras.push_back(camera);
    }
    lastSequence.assign(cameras.size(), 0);
    rotationWarned.assign(cameras.size(), false);

    // Lidar
    try {
        lidar = std::make_unique<Ouster>(config.lidarIp);
    } catch (const std::exception& e) {
        qWarning() << "Lidar not available:" << e.what();
        lidar.reset();
        connection.lidarError = e.what();
    }

    // Reset them back-to-back to sync their timers
    if (lidar) {
        lidar->resetTimestamp();
    }
    for (auto& camera : cameras) {
        camera->resetTimestamp();
    }
    onConnected(connection);
}

void Acquisition::run()
{
    connect();
    while (running) {
        // While paused the lidar is still read, so the rows restart from a fresh scan instead of a backlog
        const bool trigger = waitForTrigger();
        if (paused) {
            pending.clear();
            continue;
        }
        if (trigger) {
            startRow();
        }
        // Finish the rows in trigger order; a row waiting for its cameras never delays the next trigger
        while (!pending.empty() && rowComplete(pending.front())) {
            finishRow(pending.front());
            pending.pop_front();
        }
        if (!lidar) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

bool Acquisition::waitForTrigger()
{
    // All image collection is initiated by the lidar scan
    if (lidar) {
        return lidar->scan(LIDAR_WAIT_SEC);
    }
    // Without lidar: software trigger
    const auto now = std::chrono::steady_clock::now();
    if (now - lastSoftTrigger >= SOFT_TRIGGER_PERIOD) {
        lastSoftTrigger = now;
        return true;
    }
    return false;
}

void Acquisition::startRow()
{
    Pending row;
    row.deadline = std::chrono::steady_clock::now() + MAX_CAMERA_WAIT;
    row.frame = std::make_shared<Frame>();
    Frame& frame = *row.frame;
    frame.record = isRecording();

    if (lidar) {
        frame.cloud = lidar->getPointCloud();
        lidar->getMetaData(frame.lidar);
    } else {
        frame.lidar.ip = config.lidarIp;
    }
    frame.triggerTs = frame.lidar.ts != 0 ? frame.lidar.ts : hostTimeMicros();

    // Robot: latest state from the background poller
    frame.robot = robot->getLatest();
    if (!frame.robot.valid) {
        frame.robotError = robot->getLastError();
    }
    pending.push_back(std::move(row));
}

bool Acquisition::rowComplete(const Pending& row) const
{
    if (std::chrono::steady_clock::now() >= row.deadline) {
        return true;
    }
    // Done when every streaming camera has a frame at or after the trigger: later frames can only be further away
    const uint64_t triggerTs = row.frame->triggerTs;
    return std::all_of(cameras.begin(), cameras.end(), [triggerTs](const std::shared_ptr<Daheng>& camera) {
        return !camera->isAcquiring() || (camera->getFrameSequence() > 0 && camera->getTimeStamp() >= triggerTs);
    });
}

void Acquisition::finishRow(Pending& row)
{
    Frame& frame = *row.frame;
    frame.cameras.resize(cameras.size());
    for (size_t i = 0; i < cameras.size(); ++i) {
        Daheng& camera = *cameras[i];
        CameraSample& sample = frame.cameras[i];
        sample.ip = camera.getIp();
        sample.open = camera.isOpen();
        sample.fps = camera.getFps();
        sample.frameId = camera.getFrameId();
        sample.meta.ip = sample.ip;

        // The frame closest to the trigger, if it is close enough
        Daheng::Frame cameraFrame;
        const bool found = camera.isAcquiring() && camera.getFrameNear(frame.triggerTs, cameraFrame);
        if (!found || static_cast<uint64_t>(std::llabs(static_cast<int64_t>(cameraFrame.ts - frame.triggerTs))) > MAX_SYNC_OFFSET_US) {
            continue;
        }
        // Colour the point cloud with it (also when the previous row used it), as long as the camera is rotated as
        // during the calibration
        if (i < calibs.size()) {
            const int calibRotation = calibs[i].rotationDeg;
            if (calibRotation < 0 || calibRotation == camera.getRotation()) {
                sample.colourImage = cameraFrame.image;
            } else if (!rotationWarned[i]) {
                rotationWarned[i] = true;
                qWarning().noquote() << QString("%1 is rotated %2°, but calibrated at %3°: not used to colour the point cloud")
                    .arg(QString::fromStdString(camera.getName())).arg(camera.getRotation()).arg(calibRotation);
            }
        }
        // The frame of the row: never the one of the previous row
        if (cameraFrame.sequence > lastSequence[i]) {
            lastSequence[i] = cameraFrame.sequence;
            sample.newFrame = true;
            sample.image = cameraFrame.image;
            camera.getMetaData(cameraFrame, sample.meta);
            sample.meta.valid = true;
        }
    }
    onFrame(row.frame);
}
