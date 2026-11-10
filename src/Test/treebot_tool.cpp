// Offline tools of the treebot sensor setup, as subcommands:
//   treebot_tool snapshot <output directory>                        one lidar scan and one image per camera
//   treebot_tool segment <model dir> <out dir> <image.png> ...      segment camera images
//   treebot_tool colorize <calib.json> <cloud.ply> <out.ply> ...    colour a scan with images or classes, occupancy map, rosbag
// Run a subcommand without arguments for its options.
#include <cstring>
#include <iostream>
#include <string>

int snapshotCommand(int argc, char* argv[]);
int segmentCommand(int argc, char* argv[]);
int colorizeCommand(int argc, char* argv[]);

int main(int argc, char* argv[])
{
    const std::string command = argc > 1 ? argv[1] : "";
    // The subcommand gets the remaining arguments, with "treebot_tool <command>" as its program name
    std::string programName = std::string(argv[0]) + " " + command;
    argv[1] = programName.data();
    if (command == "snapshot") {
        return snapshotCommand(argc - 1, argv + 1);
    }
    if (command == "segment") {
        return segmentCommand(argc - 1, argv + 1);
    }
    if (command == "colorize") {
        return colorizeCommand(argc - 1, argv + 1);
    }
    std::cerr << "Usage: " << argv[0] << " <command> [arguments]\n"
              << "  snapshot   save one lidar scan and one image per camera (to check the calibration)\n"
              << "  segment    segment camera images with the segmentation model\n"
              << "  colorize   colour a recorded scan with images or classes; occupancy map (--bev), rosbag (--bag)\n";
    return command.empty() || command == "-h" || command == "--help" ? 0 : 1;
}
