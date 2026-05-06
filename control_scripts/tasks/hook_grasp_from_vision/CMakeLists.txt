cmake_minimum_required(VERSION 3.16)
project(perception_hook_testing CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(realsense2   REQUIRED)
find_package(OpenCV       REQUIRED COMPONENTS core imgproc dnn imgcodecs)
find_package(Eigen3       REQUIRED NO_MODULE)

add_executable(perception_hook_testing perception_hook_testing.cpp)

set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to ONNX Runtime install root")
if(ONNXRUNTIME_ROOT)
    set(ORT_INCLUDE_DIR "${ONNXRUNTIME_ROOT}/include")
    set(ORT_LIB_DIR "${ONNXRUNTIME_ROOT}/lib")
    find_library(ONNXRUNTIME_LIB
        NAMES onnxruntime libonnxruntime
        PATHS "${ORT_LIB_DIR}"
        NO_DEFAULT_PATH
    )
    if(NOT ONNXRUNTIME_LIB)
        message(FATAL_ERROR "ONNXRUNTIME_ROOT set but onnxruntime library not found in ${ORT_LIB_DIR}")
    endif()
    target_include_directories(perception_hook_testing PRIVATE "${ORT_INCLUDE_DIR}")
    target_link_libraries(perception_hook_testing PRIVATE "${ONNXRUNTIME_LIB}")
    target_compile_definitions(perception_hook_testing PRIVATE HAVE_ONNXRUNTIME=1)
endif()

target_link_libraries(perception_hook_testing PRIVATE
    realsense2::realsense2
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

target_include_directories(perception_hook_testing PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
