cmake_minimum_required(VERSION 3.10)
project(HandEyeCalibrationAprilTag)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# AprilTag library - you'll need to install this first
# On macOS: brew install apriltag
# On Linux: sudo apt-get install libapriltag-dev
find_library(APRILTAG_LIB apriltag REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Add executable
add_executable(calibrate_camera calibrate_camera.cpp)

# Link libraries
target_link_libraries(calibrate_camera
    ${OpenCV_LIBS}
    ${realsense2_LIBRARY}
    ${APRILTAG_LIB}
)

# Optional: Add optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(calibrate_camera PRIVATE -O3)
endif()