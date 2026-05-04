cmake_minimum_required(VERSION 3.16)
project(perception_cup CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(realsense2   REQUIRED)
find_package(OpenCV       REQUIRED COMPONENTS core imgproc dnn imgcodecs)
find_package(Eigen3       REQUIRED NO_MODULE)

add_executable(perception_cup perception.cpp)

target_link_libraries(perception_cup PRIVATE
    realsense2::realsense2
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

target_include_directories(perception_cup PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
