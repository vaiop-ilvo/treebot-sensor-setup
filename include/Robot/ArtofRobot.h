#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <sw/redis++/redis++.h>

#include <Utils/MetaData.hpp>
#include <Robot/State.hpp>

class ArtofRobot
{
private:
    sw::redis::Redis redis;

    // Background polling
    std::thread pollThread;
    std::atomic<bool> polling{false};
    std::mutex latestMutex;
    RobotMetaData latest;
    std::string lastError;

public:
    ArtofRobot(const std::string& hostname, int port=6379);
    ~ArtofRobot();

    ArtofRobot(const ArtofRobot&) = delete;
    ArtofRobot& operator=(const ArtofRobot&) = delete;

    // Reads the robot state from Redis (blocking, max ~ a few 100 ms).
    // Never throws: returns false and leaves metaData.valid = false on failure.
    bool getMetaData(RobotMetaData& metaData, std::string* error = nullptr);

    // Polls Redis in a background thread so the GUI never blocks on the network
    void startPolling(std::chrono::milliseconds period = std::chrono::milliseconds(100));
    void stopPolling();

    // Latest polled state; valid == false when the robot could not be reached
    RobotMetaData getLatest();
    std::string getLastError();
};
