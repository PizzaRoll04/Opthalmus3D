# CMakeLists.txt in src

message(STATUS "Processing src CMakeLists.txt")

# Collect all .cpp files in src and its subdirectories
file(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

# Add the executable for the project
add_executable(csv_test ${SRC_FILES})

target_link_libraries(csv_test ${OpenCV_LIBS})

# Include directories for headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR})