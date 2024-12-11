# Toolchain file for Raspberry Pi Zero 2 W
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Specify the cross-compilation tools
set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

# Specify the path to the sysroot (adjust this path to match your environment)
set(CMAKE_FIND_ROOT_PATH ~/pi_sysroot)

# Paths to custom libraries on the Pi
# Update these paths to match your environment
# set(PCL_INCLUDE_DIRS /home/divan/stereo_lidar_recorder/pcl_pi/pcl-pcl-1.12.1)
# set(PCL_INCLUDE_DIR /home/divan/stereo_lidar_recorder/pcl_pi/pcl-pcl-1.12.1/include)
# set(PCL_LIBRARY_DIR /home/divan/stereo_lidar_recorder/pcl_pi/pcl-pcl-1.12.1/lib)

# Add sysroot paths for C and C++ compilers
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# Configure search paths to prioritize cross-compiled libraries
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
