cmake_minimum_required(VERSION 3.10)
project(StereoLiDARRecorder)

# Add OpenCV paths (if required)
# set(OpenCV_DIR "~/opencv/build")
find_package(OpenCV REQUIRED)

# Add PCL
find_package(PCL REQUIRED COMPONENTS common io people)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add executable and source files
add_executable(StereoLiDARRecorder 
    src/main.cpp 
    src/StereoCamera.cpp 
    src/LiDAR.cpp
)

# Link libraries
target_link_libraries(StereoLiDARRecorder ${OpenCV_LIBS} ${PCL_LIBRARIES} pthread)
