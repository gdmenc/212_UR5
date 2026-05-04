cmake_minimum_required(VERSION 3.16)
project(calibrate_camera LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- dependencies ---------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS
    core imgproc imgcodecs highgui calib3d aruco objdetect)

find_package(realsense2 REQUIRED)

# ---- executable -----------------------------------------------------------
add_executable(calibrate_camera calibrate_camera.cpp)

target_link_libraries(calibrate_camera PRIVATE
    ${OpenCV_LIBS}
    realsense2::realsense2
)

target_include_directories(calibrate_camera PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
