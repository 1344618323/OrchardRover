# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

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
CMAKE_COMMAND = /home/cxn/clion-2020.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/cxn/clion-2020.2/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/making_map.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/making_map.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/making_map.dir/flags.make

CMakeFiles/making_map.dir/making_map.cpp.o: CMakeFiles/making_map.dir/flags.make
CMakeFiles/making_map.dir/making_map.cpp.o: ../making_map.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/making_map.dir/making_map.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/making_map.dir/making_map.cpp.o -c /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/making_map.cpp

CMakeFiles/making_map.dir/making_map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/making_map.dir/making_map.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/making_map.cpp > CMakeFiles/making_map.dir/making_map.cpp.i

CMakeFiles/making_map.dir/making_map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/making_map.dir/making_map.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/making_map.cpp -o CMakeFiles/making_map.dir/making_map.cpp.s

# Object files for target making_map
making_map_OBJECTS = \
"CMakeFiles/making_map.dir/making_map.cpp.o"

# External object files for target making_map
making_map_EXTERNAL_OBJECTS =

making_map: CMakeFiles/making_map.dir/making_map.cpp.o
making_map: CMakeFiles/making_map.dir/build.make
making_map: /usr/local/lib/libopencv_superres.so.3.4.7
making_map: /usr/local/lib/libopencv_dnn.so.3.4.7
making_map: /usr/local/lib/libopencv_objdetect.so.3.4.7
making_map: /usr/local/lib/libopencv_highgui.so.3.4.7
making_map: /usr/local/lib/libopencv_stitching.so.3.4.7
making_map: /usr/local/lib/libopencv_shape.so.3.4.7
making_map: /usr/local/lib/libopencv_ml.so.3.4.7
making_map: /usr/local/lib/libopencv_videostab.so.3.4.7
making_map: /usr/local/lib/libopencv_calib3d.so.3.4.7
making_map: /usr/local/lib/libopencv_video.so.3.4.7
making_map: /usr/local/lib/libopencv_photo.so.3.4.7
making_map: /usr/local/lib/libopencv_videoio.so.3.4.7
making_map: /usr/local/lib/libopencv_imgcodecs.so.3.4.7
making_map: /usr/local/lib/libopencv_features2d.so.3.4.7
making_map: /usr/local/lib/libopencv_flann.so.3.4.7
making_map: /usr/local/lib/libopencv_imgproc.so.3.4.7
making_map: /usr/local/lib/libopencv_core.so.3.4.7
making_map: CMakeFiles/making_map.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable making_map"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/making_map.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/making_map.dir/build: making_map

.PHONY : CMakeFiles/making_map.dir/build

CMakeFiles/making_map.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/making_map.dir/cmake_clean.cmake
.PHONY : CMakeFiles/making_map.dir/clean

CMakeFiles/making_map.dir/depend:
	cd /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug/CMakeFiles/making_map.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/making_map.dir/depend
