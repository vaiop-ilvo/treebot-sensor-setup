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

ArtofRobot::ArtofRobot(const std::string& hostname, int port, std::vector<RedisKey> keys):
    redis(makeOptions(hostname, port)),
    keys(std::move(keys))
{}

ArtofRobot::~ArtofRobot()
{
    stopPolling();
}

bool ArtofRobot::getMetaData(RobotMetaData& metaData, std::string* error)
{
    metaData = RobotMetaData{};

    try {
        // All configured keys in one round trip; reuses a pooled connection
        Pipeline pipe = redis.pipeline(false);
        for (const RedisKey& key : keys) {
            if (key.json) {
                pipe.command("JSON.GET", key.key);
            } else {
                pipe.get(key.key);
            }
        }
        auto replies = pipe.exec();
        for (size_t i = 0; i < keys.size(); ++i) {
            // nil when the key does not exist
            metaData.redis.emplace_back(keys[i].key, replies.get<std::optional<std::string>>(i));
        }
        auto value = [&metaData](const char* key) -> std::optional<std::string> {
            for (const auto& [name, raw] : metaData.redis) {
                if (name == key) {
                    return raw;
                }
            }
            return std::nullopt;
        };

        // The keys the application knows (when configured)
        metaData.gpsFix = parseOr(value("pc.gps.fix"), 0, toInt);
        metaData.gpsLngSpeed = parseOr(value("pc.gps.ground_speed"), 0.0, toDouble);
        metaData.odomLngSpeed = parseOr(value("plc.monitor.navigation.velocity.longitudinal"), 0.0, toDouble);
        metaData.odomAngSpeed = parseOr(value("plc.monitor.navigation.velocity.angular"), 0.0, toDouble);

        auto stateJson = value("gps.ref.state");
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
