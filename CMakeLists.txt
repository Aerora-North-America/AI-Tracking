cmake_minimum_required(VERSION 3.5)
project(snpe_task)

## Compile as C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -g -fPIC")

## Setup the environment, include directories and libraries 
## for the Qualcomm SNPE SDK to access the system.
set(SNPE_ROOT /home/src/snpe_2.13)
set(SNPE_INCLUDE_DIR /usr/include/zdl)
set(SNPE_LIBRARY_DIR ${SNPE_ROOT}/lib/aarch64-ubuntu-gcc7.5)

message(STATUS "SNPE root path: ${SNPE_ROOT}")
message(STATUS "SNPE include: ${SNPE_INCLUDE_DIR}")
message(STATUS "SNPE library: ${SNPE_LIBRARY_DIR}")

# if(ARM_COMPUTE_WERROR)
#   add_compile_options("-Werror")
# endif()

# # Compile with -fno-exceptions flag and define ARM_COMPUTE_EXCEPTIONS_DISABLED
# # if ARM_COMPUTE_EXCEPTIONS not set
# if(NOT ARM_COMPUTE_EXCEPTIONS)
#   add_compile_options("-fno-exceptions")
#   add_definitions(-DARM_COMPUTE_EXCEPTIONS_DISABLED)
# endif()

# # Link OpenMP libraries if ARM_COMPUTE_OPENMP set
# if(ARM_COMPUTE_OPENMP)
#   find_package(OpenMP)
#   if(OpenMP_CXX_FOUND)
#     link_libraries(OpenMP::OpenMP_CXX)
#     add_definitions(-DARM_COMPUTE_OPENMP_SCHEDULER)
#   else()
#     message(FATAL_ERROR "OPENMP was set but no OpenMP library was found!")
#   endif()
# endif()

## System dependencies are found with CMake's conventions
## the main package might be used is OpenCV and Gstreamer
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)
find_package(spdlog REQUIRED)
# find_package(GStreamer REQUIRED)

add_subdirectory(byteTrack)
add_subdirectory(mavlink)
## Add the SNPE pipeline and detection process as the static library 
## for the inference process to use
add_library(pipeline STATIC
    src/SNPEPipeline.cpp 
    src/ObjectDetector.cpp
    src/yolo.cpp)

## Add include directories of the library
target_include_directories(pipeline PUBLIC 
    include
    ${SNPE_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(pipeline PRIVATE 
    byteTrack
    mavlink
    ${OpenCV_LIBS}
    ${GFLAGS_LIBS}
    ${SNPE_ROOT}/lib/aarch64-ubuntu-gcc7.5/libSNPE.so
)

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
    include
    ${JSONCPP_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/byteTrack/include
    ${PROJECT_SOURCE_DIR}mavlink/include/v2.0
    ${PROJECT_SOURCE_DIR}mavlink/include
)

## Declare a C++ executable
## With all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
set(SOURCES main.cpp)

add_executable(snpe_task ${SOURCES})
target_link_libraries(snpe_task PRIVATE
    pipeline
    jsoncpp
    pthread
    ${GFLAGS_LIBRARIES}
    ${JSON_LIBRARIES}
)

# add_executable(snpe_task test.cpp)
# target_link_libraries(snpe_task PRIVATE
#     pipeline
#     jsoncpp
#     ${GFLAGS_LIBRARIES}
#     ${JSON_LIBRARIES}
# )