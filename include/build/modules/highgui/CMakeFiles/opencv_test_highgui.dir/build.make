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
include modules/highgui/CMakeFiles/opencv_test_highgui.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include modules/highgui/CMakeFiles/opencv_test_highgui.dir/compiler_depend.make

# Include the progress variables for this target.
include modules/highgui/CMakeFiles/opencv_test_highgui.dir/progress.make

# Include the compile flags for this target's objects.
include modules/highgui/CMakeFiles/opencv_test_highgui.dir/flags.make

modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.o: modules/highgui/CMakeFiles/opencv_test_highgui.dir/flags.make
modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.o: /home/divan/stereo_lidar_recorder/include/opencv/modules/highgui/test/test_gui.cpp
modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.o: modules/highgui/CMakeFiles/opencv_test_highgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.o"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/highgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.o -MF CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.o.d -o CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.o -c /home/divan/stereo_lidar_recorder/include/opencv/modules/highgui/test/test_gui.cpp

modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.i"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/highgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/divan/stereo_lidar_recorder/include/opencv/modules/highgui/test/test_gui.cpp > CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.i

modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.s"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/highgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/divan/stereo_lidar_recorder/include/opencv/modules/highgui/test/test_gui.cpp -o CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.s

modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.o: modules/highgui/CMakeFiles/opencv_test_highgui.dir/flags.make
modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.o: /home/divan/stereo_lidar_recorder/include/opencv/modules/highgui/test/test_main.cpp
modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.o: modules/highgui/CMakeFiles/opencv_test_highgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.o"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/highgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.o -MF CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.o.d -o CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.o -c /home/divan/stereo_lidar_recorder/include/opencv/modules/highgui/test/test_main.cpp

modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.i"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/highgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/divan/stereo_lidar_recorder/include/opencv/modules/highgui/test/test_main.cpp > CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.i

modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.s"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/highgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/divan/stereo_lidar_recorder/include/opencv/modules/highgui/test/test_main.cpp -o CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.s

# Object files for target opencv_test_highgui
opencv_test_highgui_OBJECTS = \
"CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.o" \
"CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.o"

# External object files for target opencv_test_highgui
opencv_test_highgui_EXTERNAL_OBJECTS =

bin/opencv_test_highgui: modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_gui.cpp.o
bin/opencv_test_highgui: modules/highgui/CMakeFiles/opencv_test_highgui.dir/test/test_main.cpp.o
bin/opencv_test_highgui: modules/highgui/CMakeFiles/opencv_test_highgui.dir/build.make
bin/opencv_test_highgui: lib/libopencv_ts.a
bin/opencv_test_highgui: lib/libopencv_highgui.so.4.10.0
bin/opencv_test_highgui: lib/libopencv_videoio.so.4.10.0
bin/opencv_test_highgui: 3rdparty/lib/libippiw.a
bin/opencv_test_highgui: 3rdparty/ippicv/ippicv_lnx/icv/lib/intel64/libippicv.a
bin/opencv_test_highgui: lib/libopencv_imgcodecs.so.4.10.0
bin/opencv_test_highgui: lib/libopencv_imgproc.so.4.10.0
bin/opencv_test_highgui: lib/libopencv_core.so.4.10.0
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libgtk-x11-2.0.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libgdk-x11-2.0.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libpangocairo-1.0.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libatk-1.0.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libcairo.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libgdk_pixbuf-2.0.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libgio-2.0.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libpangoft2-1.0.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libpango-1.0.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libgobject-2.0.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libglib-2.0.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libharfbuzz.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libfontconfig.so
bin/opencv_test_highgui: /usr/lib/x86_64-linux-gnu/libfreetype.so
bin/opencv_test_highgui: modules/highgui/CMakeFiles/opencv_test_highgui.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/divan/stereo_lidar_recorder/include/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../../bin/opencv_test_highgui"
	cd /home/divan/stereo_lidar_recorder/include/build/modules/highgui && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opencv_test_highgui.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
modules/highgui/CMakeFiles/opencv_test_highgui.dir/build: bin/opencv_test_highgui
.PHONY : modules/highgui/CMakeFiles/opencv_test_highgui.dir/build

modules/highgui/CMakeFiles/opencv_test_highgui.dir/clean:
	cd /home/divan/stereo_lidar_recorder/include/build/modules/highgui && $(CMAKE_COMMAND) -P CMakeFiles/opencv_test_highgui.dir/cmake_clean.cmake
.PHONY : modules/highgui/CMakeFiles/opencv_test_highgui.dir/clean

modules/highgui/CMakeFiles/opencv_test_highgui.dir/depend:
	cd /home/divan/stereo_lidar_recorder/include/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/divan/stereo_lidar_recorder/include/opencv /home/divan/stereo_lidar_recorder/include/opencv/modules/highgui /home/divan/stereo_lidar_recorder/include/build /home/divan/stereo_lidar_recorder/include/build/modules/highgui /home/divan/stereo_lidar_recorder/include/build/modules/highgui/CMakeFiles/opencv_test_highgui.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : modules/highgui/CMakeFiles/opencv_test_highgui.dir/depend

