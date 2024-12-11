cmake_minimum_required(VERSION 3.10)
project(StereoLiDARRecorder)



# Toolchain configuration for Raspberry Pi Zero 2 W
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Specify the cross-compilers for C and C++
set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

# Specify the path to the sysroot (optional, useful for library dependencies)
# set(CMAKE_SYSROOT /path/to /raspberrypi/sysroot)

# Ensure C++17 or later is used
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add OpenCV paths (if required)
# set(OpenCV_DIR "~/opencv/build")
find_package(OpenCV REQUIRED)

# Add PCL
find_package(PCL REQUIRED COMPONENTS common io people)

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "PCL_INCLUDE_DIRS1: ${PCL_INCLUDE_DIRS}")

# Set the build type
set(CMAKE_BUILD_TYPE Debug)

# Add executable and source files
# First executable: Stereo Odometry realtime
add_executable(StereoLiDARRecorder 
    src/main.cpp 
    src/StereoCamera.cpp 
    src/LiDAR.cpp
)

# Link libraries
# Link stdc++fs only for older GCC versions (7.x or 8.x)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
    target_link_libraries(StereoLiDARRecorder ${OpenCV_LIBS} ${PCL_LIBRARIES} pthread stdc++fs)
else()
    target_link_libraries(StereoLiDARRecorder ${OpenCV_LIBS} ${PCL_LIBRARIES} pthread)
endif()

# Add executable and source files
# First executable: Stereo Odometry from Images
add_executable(StereoOdomImages 
    src/StereoCamera.cpp 
    src/StereoImages.cpp
)

# Link libraries
# Link stdc++fs only for older GCC versions (7.x or 8.x)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
    target_link_libraries(StereoOdomImages ${OpenCV_LIBS} ${PCL_LIBRARIES} pthread stdc++fs)
else()
    target_link_libraries(StereoOdomImages ${OpenCV_LIBS} ${PCL_LIBRARIES} pthread)
endif()
