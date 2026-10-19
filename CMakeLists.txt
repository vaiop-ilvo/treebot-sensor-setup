cmake_minimum_required(VERSION 3.15)
project(vaiop-sensor)

set(CMAKE_CXX_STANDARD 23)

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