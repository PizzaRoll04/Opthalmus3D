cmake_minimum_required(VERSION 3.10)
project(Opthalmus3D)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find package dependencies
find_package(OpenCV REQUIRED)

# Add the include directory
include_directories(
    include
    include/opthalmus3D
    include/opthalmus3D/DataStreamer
    include/opthalmus3D/frontend
    ${OpenCV_INCLUDE_DIRS}
)

# Add subdirectories
add_subdirectory(src)