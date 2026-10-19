cmake_minimum_required(VERSION 3.5)
project(test)

##########################
##        Files         ##
##########################

set(SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(INC_DIR "${CMAKE_SOURCE_DIR}/include")

file(GLOB cameraFiles
  "${SRC_DIR}/Camera/*.cpp"
)
file(GLOB guiFiles
  "${SRC_DIR}/Gui/*.cpp"
)
# Headers with Q_OBJECT must be part of the target so AUTOMOC generates their meta code
set(mocHeaders
  "${INC_DIR}/Gui/RecordWidget.h"
)

file(GLOB lidarFiles
  "${SRC_DIR}/Lidar/*.cpp"
)

file(GLOB robotFiles
  "${SRC_DIR}/Robot/*.cpp"
)

file(GLOB utilFiles
  "${SRC_DIR}/Utils/*.cpp"
)

# Widgets shared by the small Qt examples
set(widgetFiles
  "${SRC_DIR}/Gui/Theme.cpp"
  "${SRC_DIR}/Gui/SensorHeader.cpp"
)


##########################
##        Build         ##
##########################


add_executable(camera_example "camera_example.cpp"
  "${SRC_DIR}/Camera/Daheng.cpp"
  "${SRC_DIR}/Utils/AsyncWriter.cpp"
  "${SRC_DIR}/Utils/SensorConfig.cpp"
)
target_link_libraries(camera_example
  ${DAHENG_LIBRARY}
  ${OpenCV_LIBS}
  ${Qt_LIBS}
)


add_executable(lidar_example_reflectivity "lidar_example_reflectivity.cpp"
  "${SRC_DIR}/Lidar/Ouster.cpp"
  "${SRC_DIR}/Utils/AsyncWriter.cpp"
  "${SRC_DIR}/Utils/PointCloudColor.cpp"
  "${SRC_DIR}/Utils/SensorConfig.cpp"
)
target_link_libraries(lidar_example_reflectivity
  PRIVATE OusterSDK::ouster_sensor
  ${Qt_LIBS}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

add_executable(qt_example_pointcloud "qt_example_pointcloud.cpp"
  "${SRC_DIR}/Gui/PointCloudWidgetExample.cpp"
)
target_link_libraries(qt_example_pointcloud
  ${Qt_LIBS}
)

add_executable(qt_example_lidar "qt_example_lidar.cpp"
  ${widgetFiles}
  "${SRC_DIR}/Gui/PointCloudWidget.cpp"
  "${SRC_DIR}/Gui/BirdEyeWidget.cpp"
  "${SRC_DIR}/Gui/LidarWidget.cpp"
  "${SRC_DIR}/Gui/MainWindowLidar.cpp"
  "${SRC_DIR}/Utils/GroundPlane.cpp"
  "${SRC_DIR}/Lidar/Ouster.cpp"
  "${SRC_DIR}/Utils/AsyncWriter.cpp"
  "${SRC_DIR}/Utils/PointCloudColor.cpp"
  "${SRC_DIR}/Utils/SensorConfig.cpp"
)
target_link_libraries(qt_example_lidar
  PRIVATE OusterSDK::ouster_sensor
  ${OpenCV_LIBS}
  ${Qt_LIBS}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

add_executable(qt_example_camera "qt_example_camera.cpp"
  ${widgetFiles}
  "${SRC_DIR}/Gui/ImageWidget.cpp"
  "${SRC_DIR}/Gui/CameraWidget.cpp"
  "${SRC_DIR}/Gui/MainWindowCamera.cpp"
  "${SRC_DIR}/Camera/Daheng.cpp"
  "${SRC_DIR}/Utils/AsyncWriter.cpp"
  "${SRC_DIR}/Utils/SensorConfig.cpp"
)
target_link_libraries(qt_example_camera
  ${DAHENG_LIBRARY}
  ${OpenCV_LIBS}
  ${Qt_LIBS}
)

# Main application
add_executable(qt_example_full "qt_example_full.cpp"
  ${cameraFiles}
  ${lidarFiles}
  ${guiFiles}
  ${mocHeaders}
  "${INC_DIR}/Gui/ReplayWidget.h"
  ${robotFiles}
  ${utilFiles}
)
target_link_libraries(qt_example_full
  PRIVATE OusterSDK::ouster_sensor
  ${OpenSSL_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${DAHENG_LIBRARY}
  ${OpenCV_LIBS}
  ${Qt_LIBS}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  ${HIREDIS_LIB}
  ${REDIS_PLUS_PLUS_LIB}
  ${ONNXRUNTIME_LIBRARIES}
)

add_executable(qt_example_filedialog "qt_example_filedialog.cpp"
  "${SRC_DIR}/Gui/Theme.cpp"
  "${SRC_DIR}/Gui/MainWindowFileDialog.cpp"
  "${SRC_DIR}/Gui/RecordWidget.cpp"
  ${mocHeaders}
)
target_link_libraries(qt_example_filedialog
  ${Qt_LIBS}
)

add_executable(redis_example "redis_example.cpp"
  "${SRC_DIR}/Robot/ArtofRobot.cpp"
  "${SRC_DIR}/Utils/SensorConfig.cpp"
)
target_link_libraries(redis_example
    Boost::json
    ${Qt_LIBS}
    ${OpenCV_LIBS}
    ${HIREDIS_LIB}
    ${REDIS_PLUS_PLUS_LIB}
)

add_executable(read_config "read_config.cpp"
  "${SRC_DIR}/Utils/LidarCamera.cpp"
)
target_link_libraries(read_config
  ${Qt_LIBS}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

add_executable(colorize_cloud "colorize_cloud.cpp"
  "${SRC_DIR}/Utils/GroundPlane.cpp"
  "${SRC_DIR}/Utils/LidarCamera.cpp"
  "${SRC_DIR}/Utils/PointCloudColor.cpp"
  "${SRC_DIR}/Utils/Segmenter.cpp"
)
target_link_libraries(colorize_cloud
  ${Qt_LIBS}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  ${ONNXRUNTIME_LIBRARIES}
)

add_executable(capture_snapshot "capture_snapshot.cpp"
  "${SRC_DIR}/Camera/Daheng.cpp"
  "${SRC_DIR}/Lidar/Ouster.cpp"
  "${SRC_DIR}/Utils/AsyncWriter.cpp"
  "${SRC_DIR}/Utils/PointCloudColor.cpp"
  "${SRC_DIR}/Utils/LidarCamera.cpp"
  "${SRC_DIR}/Utils/SensorConfig.cpp"
)
target_link_libraries(capture_snapshot
  PRIVATE OusterSDK::ouster_sensor
  ${DAHENG_LIBRARY}
  ${OpenCV_LIBS}
  ${Qt_LIBS}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

add_executable(segment_image "segment_image.cpp"
  "${SRC_DIR}/Utils/Segmenter.cpp"
)
target_link_libraries(segment_image
  ${Qt_LIBS}
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIBRARIES}
)
