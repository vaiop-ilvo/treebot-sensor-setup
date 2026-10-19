#include <Robot/ArtofRobot.h>
#include <Utils/MetaData.hpp>
#include <Utils/SensorConfig.h>

#include <iostream>

// Usage: redis_example [host]   (default: robot host from config/sensors.json)
int main(int argc, char* argv[])
{
    const std::string host = argc > 1 ? argv[1] : sensorConfig().robotHost;
    const int port = sensorConfig().robotPort;

    ArtofRobot robot(host, port);
    RobotMetaData metaData;
    std::string error;
    if (!robot.getMetaData(metaData, &error)) {
        std::cerr << "Error reading " << host << ":" << port << ": " << error << std::endl;
        return 1;
    }

    std::cout << "Robot MetaData:" << std::endl;
    std::cout << "Position: (" << metaData.x << ", " << metaData.y << ", " << metaData.z << ")" << std::endl;
    std::cout << "Orientation: (roll: " << metaData.roll << ", pitch: " << metaData.pitch << ", yaw: " << metaData.yaw << ")" << std::endl;
    std::cout << "Latitude: " << metaData.lat << ", Longitude: " << metaData.lon << std::endl;
    std::cout << "GPS Fix: " << metaData.gpsFix << std::endl;
    std::cout << "GPS Velocity: " << metaData.gpsLngSpeed << std::endl;
    std::cout << "Odometer Velocity: " << metaData.odomLngSpeed << std::endl;
    std::cout << "Odometer Angular Velocity: " << metaData.odomAngSpeed << std::endl;

    return 0;
}
