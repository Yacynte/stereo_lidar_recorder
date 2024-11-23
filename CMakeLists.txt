cmake_minimum_required(VERSION 3.10)
project(StereoLiDARRecorder)

# Add required libraries
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} include)

# Add source files
add_executable(StereoLiDARRecorder src/main.cpp src/StereoCamera.cpp src/LiDAR.cpp src/Utils.cpp)

# Link libraries
target_link_libraries(StereoLiDARRecorder ${OpenCV_LIBS} ${PCL_LIBRARIES})
