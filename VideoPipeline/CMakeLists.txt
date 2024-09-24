cmake_minimum_required(VERSION 3.5)

set(SNPE_ROOT /usr)
set(SNPE_INCLUDE_DIR ${SNPE_ROOT}/include/zdl)
set(SNPE_LIBRARY_DIR ${SNPE_ROOT}/lib/aarch64-ubuntu-gcc7.5)

find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)
# find_package(Streamer REQUIRED)

include(FindPkgConfig)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(JSON REQUIRED json-glib-1.0)

include_directories(
    include
    ${PROJECT_SOURCE_DIR}/snpeTask
    ${PROJECT_SOURCE_DIR}/include
    ${JSONCPP_INCLUDE_DIRS}
    ${JSON_INCLUDE_DIRS} 
    ${OPENCV_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}/
    ${GSTREAMER_APP_INCLUDE_DIRS}/
    ${GSTREAMER_RTSP_INCLUDE_DIRS}/
)

# add_subdirectory(snpeTask)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src VideoPipeLine_SRC_DIR)

add_library(VideoPipeline SHARED 
            ${VideoPipeLine_SRC_DIR}
            ${PROJECT_SOURCE_DIR}/src/ObjectDetector.cpp 
            ${PROJECT_SOURCE_DIR}/src/yolo.cpp
            )

target_link_libraries(VideoPipeline PUBLIC
    ${OpenCV_LIBRARIES}
    ${GST_LIBRARIES}
    ${GSTAPP_LIBRARIES} 
    ${GFLAGS_LIBRARIES}
    ${GSTREAMER_LIBS}
    ${JSON_LIBRARIES}
    jsoncpp
    # ${JSONCPP_LIBRARIES}
)

link_libraries(
    ${JSON_LIBRARY_DIRS}
    ${JSONCPP_LIBRARY_DIRS}
)

target_link_libraries(VideoPipeline PRIVATE
    snpeTask 
    pthread
    ${SNPE_ROOT}/lib/aarch64-ubuntu-gcc7.5/libSNPE.so
)

target_include_directories(VideoPipeline PUBLIC  
${SNPE_INCLUDE_DIR}    
${SNPE_INCLUDE_DIR}/DlContainer
${SNPE_INCLUDE_DIR}/DlSystem
${SNPE_INCLUDE_DIR}/SNPE
${SNPE_INCLUDE_DIR}/DiagLog)
