cmake_minimum_required(VERSION 2.6)
set( CMAKE_CXX_FLAGS "-g -Wall -std=c++11 ")
project(tinyrenderer)

include_directories(
  lib/
  common/
)

link_directories(
    ${PROJECT_SOURCE_DIR}/lib
        ${PROJECT_SOURCE_DIR}/common
)

add_definitions(-DSRC_PATH=\"${CMAKE_SOURCE_DIR}/\")

add_executable(tinyrenderer
        main.cpp
        
        #ssloy files
        tgaimage.cpp
        model.cpp
        geometry.cpp
)
target_link_libraries(tinyrenderer ${CL_LIBRARIES})
