#include <Robot/ArtofRobot.h>
#include <boost/json.hpp>

using namespace sw::redis;

namespace {
    ConnectionOptions makeOptions(const std::string& hostname, int port)
    {
        ConnectionOptions opts;
        opts.host = hostname;
        opts.port = port;
        // Never hang the application when the robot is not reachable
        opts.connect_timeout = std::chrono::milliseconds(300);
        opts.socket_timeout = std::chrono::milliseconds(300);
        return opts;
    }

    template <typename T, typename Parse>
    T parseOr(const std::optional<std::string>& value, T fallback, Parse parse)
    {
        if (!value) {
            return fallback;
        }
        try {
            return parse(*value);
        } catch (const std::exception&) {
            return fallback;
        }
    }

    double toDouble(const std::string& s) { return std::stod(s); }
    int toInt(const std::string& s) { return std::stoi(s); }
}

ArtofRobot::ArtofRobot(const std::string& hostname, int port):
    redis(makeOptions(hostname, port))
{}

ArtofRobot::~ArtofRobot()
{
    stopPolling();
}

bool ArtofRobot::getMetaData(RobotMetaData& metaData, std::string* error)
{
    metaData = RobotMetaData{};

    try {
        // Reuse a pooled connection instead of opening a new one for every call
        Pipeline pipe = redis.pipeline(false);
        pipe.get("pc.gps.fix");
        pipe.get("pc.gps.ground_speed");
        pipe.get("plc.monitor.navigation.velocity.longitudinal");
        pipe.get("plc.monitor.navigation.velocity.angular");
        pipe.command("JSON.GET", "gps.ref.state");

        auto replies = pipe.exec();

        // Retrieve in the same order
        metaData.gpsFix = parseOr(replies.get<std::optional<std::string>>(0), 0, toInt);
        metaData.gpsLngSpeed = parseOr(replies.get<std::optional<std::string>>(1), 0.0, toDouble);
        metaData.odomLngSpeed = parseOr(replies.get<std::optional<std::string>>(2), 0.0, toDouble);
        metaData.odomAngSpeed = parseOr(replies.get<std::optional<std::string>>(3), 0.0, toDouble);

        // JSON.GET returns nil when the key does not exist
        auto stateJson = replies.get<std::optional<std::string>>(4);
        if (stateJson && !stateJson->empty())
        {
            State state = boost::json::value_to<State>(boost::json::parse(*stateJson));

            metaData.x = state.T[0];
            metaData.y = state.T[1];
            metaData.z = state.T[2];

            metaData.roll  = state.R[0];
            metaData.pitch = state.R[1];
            metaData.yaw   = state.R[2];

            metaData.lat = state.point.latlng[0];
            metaData.lon = state.point.latlng[1];
        }

        metaData.valid = true;
        return true;
    } catch (const std::exception& e) {
        if (error) {
            *error = e.what();
        }
        return false;
    }
}

void ArtofRobot::startPolling(std::chrono::milliseconds period)
{
    if (polling.exchange(true)) {
        return;
    }

    pollThread = std::thread([this, period]() {
        while (polling.load()) {
            auto next = std::chrono::steady_clock::now() + period;

            RobotMetaData data;
            std::string error;
            getMetaData(data, &error);
            {
                std::lock_guard<std::mutex> lock(latestMutex);
                latest = data;
                lastError = error;
            }

            // Sleep in small steps so stopPolling() returns quickly
            while (polling.load() && std::chrono::steady_clock::now() < next) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    });
}

void ArtofRobot::stopPolling()
{
    polling.store(false);
    if (pollThread.joinable()) {
        pollThread.join();
    }
}

RobotMetaData ArtofRobot::getLatest()
{
    std::lock_guard<std::mutex> lock(latestMutex);
    return latest;
}

std::string ArtofRobot::getLastError()
{
    std::lock_guard<std::mutex> lock(latestMutex);
    return lastError;
}
