# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/divan/stereo_lidar_recorder/include/opencv

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/divan/stereo_lidar_recorder/include/build

# Include any dependencies generated for this target.
include modules/ts/CMakeFiles/opencv_ts.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include modules/ts/CMakeFiles/opencv_ts.dir/compiler_depend.make

# Include the progress variables for this target.
include modules/ts/CMakeFiles/opencv_ts.dir/progress.make

# Include the compile flags for this target's objects.
include modules/ts/CMakeFiles/opencv_ts.dir/flags.make

modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/flags.make
modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.o: /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/cuda_perf.cpp
modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.o"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.o -MF CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.o.d -o CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.o -c /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/cuda_perf.cpp

modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.i"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/cuda_perf.cpp > CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.i

modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.s"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/cuda_perf.cpp -o CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.s

modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/flags.make
modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.o: /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/cuda_test.cpp
modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.o"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.o -MF CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.o.d -o CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.o -c /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/cuda_test.cpp

modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.i"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/cuda_test.cpp > CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.i

modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.s"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/cuda_test.cpp -o CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.s

modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/flags.make
modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.o: /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ocl_perf.cpp
modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.o"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.o -MF CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.o.d -o CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.o -c /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ocl_perf.cpp

modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.i"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ocl_perf.cpp > CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.i

modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.s"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ocl_perf.cpp -o CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.s

modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/flags.make
modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.o: /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ocl_test.cpp
modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.o"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.o -MF CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.o.d -o CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.o -c /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ocl_test.cpp

modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.i"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ocl_test.cpp > CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.i

modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.s"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ocl_test.cpp -o CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.s

modules/ts/CMakeFiles/opencv_ts.dir/src/ts.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/flags.make
modules/ts/CMakeFiles/opencv_ts.dir/src/ts.cpp.o: /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts.cpp
modules/ts/CMakeFiles/opencv_ts.dir/src/ts.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object modules/ts/CMakeFiles/opencv_ts.dir/src/ts.cpp.o"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT modules/ts/CMakeFiles/opencv_ts.dir/src/ts.cpp.o -MF CMakeFiles/opencv_ts.dir/src/ts.cpp.o.d -o CMakeFiles/opencv_ts.dir/src/ts.cpp.o -c /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts.cpp

modules/ts/CMakeFiles/opencv_ts.dir/src/ts.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_ts.dir/src/ts.cpp.i"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts.cpp > CMakeFiles/opencv_ts.dir/src/ts.cpp.i

modules/ts/CMakeFiles/opencv_ts.dir/src/ts.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_ts.dir/src/ts.cpp.s"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts.cpp -o CMakeFiles/opencv_ts.dir/src/ts.cpp.s

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/flags.make
modules/ts/CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.o: /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_arrtest.cpp
modules/ts/CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object modules/ts/CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.o"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT modules/ts/CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.o -MF CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.o.d -o CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.o -c /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_arrtest.cpp

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.i"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_arrtest.cpp > CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.i

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.s"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_arrtest.cpp -o CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.s

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_func.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/flags.make
modules/ts/CMakeFiles/opencv_ts.dir/src/ts_func.cpp.o: /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_func.cpp
modules/ts/CMakeFiles/opencv_ts.dir/src/ts_func.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object modules/ts/CMakeFiles/opencv_ts.dir/src/ts_func.cpp.o"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT modules/ts/CMakeFiles/opencv_ts.dir/src/ts_func.cpp.o -MF CMakeFiles/opencv_ts.dir/src/ts_func.cpp.o.d -o CMakeFiles/opencv_ts.dir/src/ts_func.cpp.o -c /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_func.cpp

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_func.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_ts.dir/src/ts_func.cpp.i"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_func.cpp > CMakeFiles/opencv_ts.dir/src/ts_func.cpp.i

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_func.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_ts.dir/src/ts_func.cpp.s"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_func.cpp -o CMakeFiles/opencv_ts.dir/src/ts_func.cpp.s

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/flags.make
modules/ts/CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.o: /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_gtest.cpp
modules/ts/CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object modules/ts/CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.o"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT modules/ts/CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.o -MF CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.o.d -o CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.o -c /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_gtest.cpp

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.i"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_gtest.cpp > CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.i

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.s"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_gtest.cpp -o CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.s

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/flags.make
modules/ts/CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.o: /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_perf.cpp
modules/ts/CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object modules/ts/CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.o"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT modules/ts/CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.o -MF CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.o.d -o CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.o -c /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_perf.cpp

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.i"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_perf.cpp > CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.i

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.s"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_perf.cpp -o CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.s

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/flags.make
modules/ts/CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.o: /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_tags.cpp
modules/ts/CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.o: modules/ts/CMakeFiles/opencv_ts.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object modules/ts/CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.o"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT modules/ts/CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.o -MF CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.o.d -o CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.o -c /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_tags.cpp

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.i"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_tags.cpp > CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.i

modules/ts/CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.s"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/divan/stereo_lidar_recorder/include/opencv/modules/ts/src/ts_tags.cpp -o CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.s

# Object files for target opencv_ts
opencv_ts_OBJECTS = \
"CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.o" \
"CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.o" \
"CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.o" \
"CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.o" \
"CMakeFiles/opencv_ts.dir/src/ts.cpp.o" \
"CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.o" \
"CMakeFiles/opencv_ts.dir/src/ts_func.cpp.o" \
"CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.o" \
"CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.o" \
"CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.o"

# External object files for target opencv_ts
opencv_ts_EXTERNAL_OBJECTS =

lib/libopencv_ts.a: modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_perf.cpp.o
lib/libopencv_ts.a: modules/ts/CMakeFiles/opencv_ts.dir/src/cuda_test.cpp.o
lib/libopencv_ts.a: modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_perf.cpp.o
lib/libopencv_ts.a: modules/ts/CMakeFiles/opencv_ts.dir/src/ocl_test.cpp.o
lib/libopencv_ts.a: modules/ts/CMakeFiles/opencv_ts.dir/src/ts.cpp.o
lib/libopencv_ts.a: modules/ts/CMakeFiles/opencv_ts.dir/src/ts_arrtest.cpp.o
lib/libopencv_ts.a: modules/ts/CMakeFiles/opencv_ts.dir/src/ts_func.cpp.o
lib/libopencv_ts.a: modules/ts/CMakeFiles/opencv_ts.dir/src/ts_gtest.cpp.o
lib/libopencv_ts.a: modules/ts/CMakeFiles/opencv_ts.dir/src/ts_perf.cpp.o
lib/libopencv_ts.a: modules/ts/CMakeFiles/opencv_ts.dir/src/ts_tags.cpp.o
lib/libopencv_ts.a: modules/ts/CMakeFiles/opencv_ts.dir/build.make
lib/libopencv_ts.a: modules/ts/CMakeFiles/opencv_ts.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX static library ../../lib/libopencv_ts.a"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && $(CMAKE_COMMAND) -P CMakeFiles/opencv_ts.dir/cmake_clean_target.cmake
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opencv_ts.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
modules/ts/CMakeFiles/opencv_ts.dir/build: lib/libopencv_ts.a
.PHONY : modules/ts/CMakeFiles/opencv_ts.dir/build

modules/ts/CMakeFiles/opencv_ts.dir/clean:
	cd /home/divan/stereo_lidar_recorder/include/build/modules/ts && $(CMAKE_COMMAND) -P CMakeFiles/opencv_ts.dir/cmake_clean.cmake
.PHONY : modules/ts/CMakeFiles/opencv_ts.dir/clean

modules/ts/CMakeFiles/opencv_ts.dir/depend:
	cd /home/divan/stereo_lidar_recorder/include/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/divan/stereo_lidar_recorder/include/opencv /home/divan/stereo_lidar_recorder/include/opencv/modules/ts /home/divan/stereo_lidar_recorder/include/build /home/divan/stereo_lidar_recorder/include/build/modules/ts /home/divan/stereo_lidar_recorder/include/build/modules/ts/CMakeFiles/opencv_ts.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : modules/ts/CMakeFiles/opencv_ts.dir/depend

