cmake_minimum_required(VERSION 3.14)
project(CogipAPI)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Python packages
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(PythonLibs REQUIRED)

# Find Cython .cmake files
file(GLOB_RECURSE FIND_CYTHON_CMAKE_FILE "${PROJECT_SOURCE_DIR}/*Cython.cmake")
get_filename_component(FIND_CYTHON_CMAKE_DIR "${FIND_CYTHON_CMAKE_FILE}" DIRECTORY)
list( APPEND CMAKE_MODULE_PATH ${FIND_CYTHON_CMAKE_DIR} )
# Cython packages
find_package(Cython MODULE REQUIRED VERSION 3.0)
include(UseCython)

# Include directories
include_directories(${PYTHON_INCLUDE_DIRS})
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
    lib/obstacles/ObstacleCircle.cpp
    lib/obstacles/Obstacle.cpp
    lib/obstacles/ObstaclePolygon.cpp
    lib/obstacles/ObstacleRectangle.cpp
    lib/path/Pose.cpp
    lib/utils/utils.cpp
)

# Create shared library libavoidance.so
add_library(avoidance_lib SHARED ${SOURCES})

# Test executables
add_executable(test_avoidance applications/tests/avoidance/main.cpp)

# Link the test executable with the avoidance library
target_link_libraries(test_avoidance avoidance_lib)

# Set output directories for binaries and libraries
set_target_properties(avoidance_lib PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)
set_target_properties(test_avoidance PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Define the Python module directory (where your .pyx file is located)
set(PYTHON_MODULE_DIR ${CMAKE_SOURCE_DIR}/applications/tests/avoidance)
# Define the .pyx file to be compiled
set(PYTHON_MODULE_SRC ${PYTHON_MODULE_DIR}/pyavoidance.pyx)

# Add the cython transpile step
cython_transpile(${PYTHON_MODULE_SRC}
                LANGUAGE CXX
                CYTHON_ARGS
                    --cplus -3
                OUTPUT_VARIABLE PYAVOIDANCE_CPP)

# Define the Python extension using Cython
python_add_library(pyavoidance MODULE WITH_SOABI ${PYAVOIDANCE_CPP})

# Set output directory for the shared library (.so)
set_target_properties(pyavoidance PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python_build/
)

# Link C++ libraries to the Cython module
target_link_libraries(pyavoidance PRIVATE avoidance_lib Python::Module)
