#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Camera/Daheng.h>
#include <Lidar/Ouster.h>
#include <Robot/ArtofRobot.h>
#include <Utils/Frame.h>
#include <Utils/LidarCamera.h>
#include <Utils/SensorConfig.h>

// Connects the sensors and makes the rows, on its own thread: every new lidar scan is a trigger (without lidar a
// 10 Hz timer); for each camera the frame closest to the trigger (at most 50 ms away, never the same frame in two
// rows) and the latest robot state complete the row. Rows are emitted in trigger order, as soon as every streaming
// camera has a frame at or after the trigger (at most 150 ms later), also while later triggers are waiting.
class Acquisition {
public:
    // Result of connecting the sensors (emitted once, on the acquisition thread)
    struct Connection {
        std::vector<std::string> cameraErrors;      // Per camera, empty when it is open
        std::vector<int> cameraRotations;
        std::string lidarError;                     // Empty when the lidar is connected
    };

    // isRecording: asked at every trigger; onFrame and onConnected are called on the acquisition thread
    Acquisition(const SensorConfig& config, std::vector<CameraCalib> calibs, std::function<bool()> isRecording,
                std::function<void(std::shared_ptr<const Frame>)> onFrame, std::function<void(Connection)> onConnected);
    ~Acquisition();

    // Connects the sensors and starts making rows
    void start();
    // Stops the thread and the sensors
    void stop();
    // While paused (replay) the lidar is still read, so the rows restart from a fresh scan, but no rows are made
    void setPaused(bool paused);
    // Rotates a camera 90 degrees clockwise; returns the new rotation (degrees)
    int rotateCamera(size_t index);

private:
    struct Pending {
        std::chrono::steady_clock::time_point deadline;
        std::shared_ptr<Frame> frame;
    };

    const SensorConfig config;
    const std::vector<CameraCalib> calibs;
    const std::function<bool()> isRecording;
    const std::function<void(std::shared_ptr<const Frame>)> onFrame;
    const std::function<void(Connection)> onConnected;

    std::mutex camerasMutex;                        // cameras is filled by the thread, rotated by the GUI
    std::vector<std::shared_ptr<Daheng>> cameras;
    std::unique_ptr<Ouster> lidar;
    std::unique_ptr<ArtofRobot> robot;
    std::atomic<bool> running{false};
    std::atomic<bool> paused{false};
    std::thread thread;

    // Acquisition thread only
    std::deque<Pending> pending;
    std::vector<uint64_t> lastSequence;             // Frame of the previous row per camera
    std::vector<bool> rotationWarned;
    std::chrono::steady_clock::time_point lastSoftTrigger;

    void run();
    void connect();
    bool waitForTrigger();
    void startRow();
    bool rowComplete(const Pending& row) const;
    void finishRow(Pending& row);
};
