cmake_minimum_required(VERSION 3.15)
project(vaiop-sensor)

set(CMAKE_CXX_STANDARD 23)

# Release unless asked otherwise: the per-point loops (point cloud colouring) are ~100x slower in Debug
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif ()

# Write build/compile_commands.json for IntelliSense / clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

IF (NOT DEFINED CMAKE_INSTALL_LIBDIR)
    SET(CMAKE_INSTALL_LIBDIR lib)
ENDIF ()

SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY
        ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}
        CACHE PATH
        ""
    )

SET (CMAKE_RUNTIME_OUTPUT_DIRECTORY
        ${PROJECT_BINARY_DIR}/bin
        CACHE PATH
        ""
    )

SET (CMAKE_ARCHIVE_OUTPUT_DIRECTORY
        ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}
        CACHE PATH
        ""
    )

## Compile with ASAN
# Off by default. Enable with `cmake -DENABLE_ASAN=ON ..` (or the environment variable NO_ASAN=0).
option(ENABLE_ASAN "Compile with the address sanitizer" OFF)
if ("$ENV{NO_ASAN}" STREQUAL "0")
    set(ENABLE_ASAN ON)
endif()
if(NOT WIN32 AND ENABLE_ASAN)
    message(STATUS "Note: Compiling with asan address sanitization and leak checking")
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    # Eigen uses its own aligned malloc/free under ASAN, but the (uninstrumented) Ouster SDK returns matrices
    # allocated with plain malloc: freeing them here would be reported as a heap-buffer-overflow
    add_compile_definitions(EIGEN_MALLOC_ALREADY_ALIGNED=1)
    add_link_options(-fsanitize=address)
else()
    message(STATUS "ASAN disabled: to enable, run: cmake -DENABLE_ASAN=ON ..")
endif()

## Ignore BOOST deprecated headers
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")

# Find Package Dependencies
# Find Boost
find_package(Boost REQUIRED COMPONENTS json system filesystem context thread)

# Native installation
# find_package(Boost REQUIRED filesystem system thread context REQUIRED)

find_package(OpenSSL REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Redis++
find_path(HIREDIS_HEADER hiredis)
find_library(HIREDIS_LIB hiredis)

find_path(REDIS_PLUS_PLUS_HEADER sw)
find_library(REDIS_PLUS_PLUS_LIB redis++)

# Find Ouster SKD
find_package(OusterSDK REQUIRED)

# Find Point Cloud Library (PCL)
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
find_package(PCL REQUIRED io)
find_package(Eigen3 REQUIRED)

# ONNX Runtime (optional): colouring the point cloud with a segmentation model.
# Unpack a release from https://github.com/microsoft/onnxruntime/releases in thirdparty/onnxruntime.
set(ONNXRUNTIME_ROOT "${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime" CACHE PATH "ONNX Runtime folder (include/, lib/)")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS "${ONNXRUNTIME_ROOT}/include" PATH_SUFFIXES onnxruntime onnxruntime/core/session)
find_library(ONNXRUNTIME_LIB onnxruntime HINTS "${ONNXRUNTIME_ROOT}/lib")
if (ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
    message(STATUS "ONNX Runtime found: ${ONNXRUNTIME_LIB} (point cloud segmentation enabled)")
    add_compile_definitions(WITH_SEGMENTATION)
    # CUDA / cuDNN libraries for the GPU build of ONNX Runtime (scripts/download_models.sh --gpu); loaded at run time
    set(SEGMENTATION_CUDA_DIR "${PROJECT_SOURCE_DIR}/thirdparty/cuda/lib" CACHE PATH "CUDA 12 / cuDNN 9 libraries for ONNX Runtime GPU")
    add_compile_definitions(SEGMENTATION_CUDA_DIR="${SEGMENTATION_CUDA_DIR}")
    if (EXISTS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime_providers_cuda.so")
        message(STATUS "ONNX Runtime is the GPU build: segmentation can run on the GPU (CUDA)")
    endif ()
    include_directories(${ONNXRUNTIME_INCLUDE_DIR})
    set(ONNXRUNTIME_LIBRARIES ${ONNXRUNTIME_LIB})
else ()
    message(STATUS "ONNX Runtime not found in ${ONNXRUNTIME_ROOT}: point cloud segmentation disabled")
    set(ONNXRUNTIME_LIBRARIES "")
endif ()

# ROS 2 (optional): recording to a rosbag (rosbag2, MCAP storage). Found in ROS_PREFIX, no need to source setup.bash.
set(ROS_PREFIX "/opt/ros/jazzy" CACHE PATH "ROS 2 installation used for the rosbag recording")
set(ROSBAG_LIBRARIES "")
if (EXISTS "${ROS_PREFIX}")
    list(APPEND CMAKE_PREFIX_PATH "${ROS_PREFIX}")
    # What setup.bash would set: ament's CMake code runs Python scripts that need ament_package
    file(GLOB ROS_PYTHON_PATH "${ROS_PREFIX}/lib/python3*/site-packages")
    set(ENV{PYTHONPATH} "${ROS_PYTHON_PATH}:$ENV{PYTHONPATH}")
    set(ENV{AMENT_PREFIX_PATH} "${ROS_PREFIX}")
    find_package(rclcpp QUIET)
    find_package(rosbag2_cpp QUIET)
    find_package(std_msgs QUIET)
    find_package(geometry_msgs QUIET)
    find_package(sensor_msgs QUIET)
    find_package(nav_msgs QUIET)
    find_package(tf2_msgs QUIET)
    find_package(grid_map_msgs QUIET)
endif ()
if (rclcpp_FOUND AND rosbag2_cpp_FOUND AND std_msgs_FOUND AND geometry_msgs_FOUND AND sensor_msgs_FOUND
        AND nav_msgs_FOUND AND tf2_msgs_FOUND)
    message(STATUS "ROS 2 found in ${ROS_PREFIX}: recording to a rosbag enabled")
    add_compile_definitions(WITH_ROSBAG ROS_PREFIX="${ROS_PREFIX}")
    # The ROS libraries have no RUNPATH of their own: setup.bash puts them on LD_LIBRARY_PATH. An old-style RPATH
    # (DT_RPATH, unlike DT_RUNPATH also used for their dependencies and for the plugins rosbag2 loads) lets the
    # programs run without sourcing setup.bash. See ROSBAG_LINK_OPTIONS in src/Test/CMakeLists.txt.
    set(ROSBAG_LINK_OPTIONS "-Wl,--disable-new-dtags" "-Wl,-rpath,${ROS_PREFIX}/lib")
    set(ROSBAG_LIBRARIES rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp ${std_msgs_TARGETS} ${geometry_msgs_TARGETS}
        ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${tf2_msgs_TARGETS})
    if (grid_map_msgs_FOUND)
        message(STATUS "grid_map_msgs found: the occupancy map is also recorded as a GridMap (with the class layer)")
        add_compile_definitions(WITH_GRID_MAP)
        list(APPEND ROSBAG_LIBRARIES ${grid_map_msgs_TARGETS})
    else ()
        message(STATUS "grid_map_msgs not found (sudo apt install ros-jazzy-grid-map-msgs): no GridMap in the rosbag")
    endif ()
else ()
    message(STATUS "ROS 2 not found in ${ROS_PREFIX}: recording to a rosbag disabled")
endif ()

# Find Qt5 
find_package(Qt5 REQUIRED COMPONENTS Core Gui OpenGL Widgets REQUIRED)
# Enable automatic MOC, UIC, and RCC processing
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Link Daheng cameras
set(DAHENG_LIBRARY 
    "/usr/lib/libgxiapi.so"
)

set(Qt_LIBS
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
)

set(OpenSSL_LIBRARIES
    OpenSSL::SSL
    OpenSSL::Crypto
)

set(BOOST_LIBRARIES
    Boost::system
    Boost::context
    Boost::thread
    Boost::json
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    "${PROJECT_SOURCE_DIR}/thirdparty/Galaxy_camera/inc"
    "${PROJECT_SOURCE_DIR}/thirdparty/ouster-sdk/ouster_viz/include"
    ${OpenCV_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${X11_INCLUDE_DIR}
    ${BOOST_INCLUDEDIR}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

link_directories(${BOOST_LIBRARYDIR})
# message("ADDITIONAL_PUBLIC_COMPILE_OPTIONS: ${ADDITIONAL_PUBLIC_COMPILE_OPTIONS}")
# message("ADDITIONAL_LINK_LIBRARIES: ${ADDITIONAL_LINK_LIBRARIES}")

##########################
## Include directories  ##
##########################

# Libraries
# add_subdirectory(src/ThirdParty)

# Executables
# Core functionality
add_subdirectory(src/Test)

# if(NOT UTEST)
#     message("-- Testing disabled")
# else()
#     message("-- Generating test files ...")
#     add_subdirectory(src/UnitTest)
# endif()