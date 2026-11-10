cmake_minimum_required(VERSION 3.5)
project(test)

set(SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(INC_DIR "${CMAKE_SOURCE_DIR}/include")

# Sensors, processing and recording: everything except the GUI, shared by the application and the tools
file(GLOB coreFiles
  "${SRC_DIR}/Camera/*.cpp"
  "${SRC_DIR}/Lidar/*.cpp"
  "${SRC_DIR}/Robot/*.cpp"
  "${SRC_DIR}/Utils/*.cpp"
)
add_library(treebot_core STATIC ${coreFiles})
target_link_libraries(treebot_core PUBLIC
  OusterSDK::ouster_sensor
  ${OpenSSL_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${DAHENG_LIBRARY}
  ${OpenCV_LIBS}
  ${Qt_LIBS}
  Eigen3::Eigen
  ${HIREDIS_LIB}
  ${REDIS_PLUS_PLUS_LIB}
  ${ONNXRUNTIME_LIBRARIES}
  ${ROSBAG_LIBRARIES}
)

# Main application. Headers with Q_OBJECT must be part of the target so AUTOMOC generates their meta code.
file(GLOB guiFiles "${SRC_DIR}/Gui/*.cpp")
add_executable(qt_example_full "qt_example_full.cpp"
  ${guiFiles}
  "${INC_DIR}/Gui/RecordWidget.h"
  "${INC_DIR}/Gui/ReplayWidget.h"
)
target_link_libraries(qt_example_full PRIVATE treebot_core)

# Offline tools: treebot_tool snapshot | segment | colorize
add_executable(treebot_tool "treebot_tool.cpp" "tool_snapshot.cpp" "tool_segment.cpp" "tool_colorize.cpp")
target_link_libraries(treebot_tool PRIVATE treebot_core)

# Programs that use ROS 2: find its libraries without setup.bash (see ROSBAG_LINK_OPTIONS in the top CMakeLists.txt)
if (ROSBAG_LINK_OPTIONS)
  target_link_options(qt_example_full PRIVATE ${ROSBAG_LINK_OPTIONS})
  target_link_options(treebot_tool PRIVATE ${ROSBAG_LINK_OPTIONS})
endif ()
