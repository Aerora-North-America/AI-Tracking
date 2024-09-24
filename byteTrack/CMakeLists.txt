cmake_minimum_required(VERSION 3.2.0)

# project(byty)

# include(FindPkgConfig)

# pkg_check_modules(JSONCPP REQUIRED jsoncpp)
# pkg_check_modules(JSON    REQUIRED json-glib-1.0)

set(PROJECT_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_MODULE_PATH ${PROJECT_ROOT}/CMake)
set(CMAKE_CXX_STANDARD 11)

# find_package(GStreamer REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include
    /usr/include
    )
# add_subdirectory("./src")

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src BYTETRACK_SRCS)

add_library(byteTrack SHARED ${BYTETRACK_SRCS})


target_link_libraries(byteTrack PUBLIC ${OpenCV_LIBS}  )
# link_directories(
#     ${JSONCPP_LIBRARY_DIRS} 
#     ${JSON_LIBRARY_DIRS}
# )