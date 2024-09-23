cmake_minimum_required(VERSION 3.14)
project(CogipAPI)

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

# Create shared library libavoidance.so
add_library(avoidance SHARED ${SOURCES})

# Test executables
add_executable(test_avoidance applications/tests/avoidance/main.cpp)

# Link the test executable with the avoidance library
target_link_libraries(test_avoidance avoidance)

# Set output directories for binaries and libraries
set_target_properties(avoidance PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)
set_target_properties(test_avoidance PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
