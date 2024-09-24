
cmake_minimum_required(VERSION 3.5)

# project(DeepSORT LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package (Eigen3 REQUIRED NO_MODULE)

set(ONNXRUNTIME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/onnxruntime-linux-aarch64-1.12.1)

message(STATUS "onnxruntime_dir: ${ONNXRUNTIME_DIR}")

include_directories(${ONNXRUNTIME_DIR}/include
    )

find_package(OpenCV 4 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src DEEPSORT_SRC_DIR)

add_library(deepsort SHARED ${DEEPSORT_SRC_DIR})

target_link_libraries(deepsort PUBLIC  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so ${OpenCV_LIBS} Eigen3::Eigen)

