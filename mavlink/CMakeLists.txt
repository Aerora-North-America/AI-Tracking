aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src mavlink_src)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/v2.0
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include
    )

# target_link_directories(
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/include/v2.0
# )    

add_library(mavlink 
SHARED 
${mavlink_src})
