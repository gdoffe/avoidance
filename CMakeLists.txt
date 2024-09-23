cmake_minimum_required(VERSION 3.14)
project(CogipAPI)

# Configure pybind11
find_package(pybind11 REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/lib/avoidance/include
    ${CMAKE_SOURCE_DIR}/lib/cogip_defs/include
    ${CMAKE_SOURCE_DIR}/lib/logger/include
    ${CMAKE_SOURCE_DIR}/lib/obstacles/include
    ${CMAKE_SOURCE_DIR}/lib/path/include
    ${CMAKE_SOURCE_DIR}/lib/trigonometry/include
    ${CMAKE_SOURCE_DIR}/lib/utils/include
)

# Source files
set(SOURCES
    lib/avoidance/Avoidance.cpp
    lib/cogip_defs/Coords.cpp
    lib/obstacles/Circle.cpp
    lib/obstacles/Obstacle.cpp
    lib/obstacles/ObstaclePolygon.cpp
    lib/obstacles/ObstacleRectangle.cpp
    lib/path/Pose.cpp
    lib/utils/utils.cpp
)

# Create the Python module
pybind11_add_module(cogip_api MODULE ${SOURCES})

