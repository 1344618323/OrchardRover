# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/cxn/software/clion-2019.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/cxn/software/clion-2019.2.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/making_maps2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/making_maps2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/making_maps2.dir/flags.make

CMakeFiles/making_maps2.dir/making_maps2.cpp.o: CMakeFiles/making_maps2.dir/flags.make
CMakeFiles/making_maps2.dir/making_maps2.cpp.o: ../making_maps2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/making_maps2.dir/making_maps2.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/making_maps2.dir/making_maps2.cpp.o -c /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/making_maps2.cpp

CMakeFiles/making_maps2.dir/making_maps2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/making_maps2.dir/making_maps2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/making_maps2.cpp > CMakeFiles/making_maps2.dir/making_maps2.cpp.i

CMakeFiles/making_maps2.dir/making_maps2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/making_maps2.dir/making_maps2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/making_maps2.cpp -o CMakeFiles/making_maps2.dir/making_maps2.cpp.s

# Object files for target making_maps2
making_maps2_OBJECTS = \
"CMakeFiles/making_maps2.dir/making_maps2.cpp.o"

# External object files for target making_maps2
making_maps2_EXTERNAL_OBJECTS =

making_maps2: CMakeFiles/making_maps2.dir/making_maps2.cpp.o
making_maps2: CMakeFiles/making_maps2.dir/build.make
making_maps2: /usr/local/lib/libopencv_stitching.so.3.4.7
making_maps2: /usr/local/lib/libopencv_shape.so.3.4.7
making_maps2: /usr/local/lib/libopencv_highgui.so.3.4.7
making_maps2: /usr/local/lib/libopencv_dnn.so.3.4.7
making_maps2: /usr/local/lib/libopencv_objdetect.so.3.4.7
making_maps2: /usr/local/lib/libopencv_superres.so.3.4.7
making_maps2: /usr/local/lib/libopencv_ml.so.3.4.7
making_maps2: /usr/local/lib/libopencv_videostab.so.3.4.7
making_maps2: /usr/local/lib/libopencv_calib3d.so.3.4.7
making_maps2: /usr/local/lib/libopencv_features2d.so.3.4.7
making_maps2: /usr/local/lib/libopencv_flann.so.3.4.7
making_maps2: /usr/local/lib/libopencv_photo.so.3.4.7
making_maps2: /usr/local/lib/libopencv_video.so.3.4.7
making_maps2: /usr/local/lib/libopencv_videoio.so.3.4.7
making_maps2: /usr/local/lib/libopencv_imgcodecs.so.3.4.7
making_maps2: /usr/local/lib/libopencv_imgproc.so.3.4.7
making_maps2: /usr/local/lib/libopencv_core.so.3.4.7
making_maps2: CMakeFiles/making_maps2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable making_maps2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/making_maps2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/making_maps2.dir/build: making_maps2

.PHONY : CMakeFiles/making_maps2.dir/build

CMakeFiles/making_maps2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/making_maps2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/making_maps2.dir/clean

CMakeFiles/making_maps2.dir/depend:
	cd /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug /home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/cmake-build-debug/CMakeFiles/making_maps2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/making_maps2.dir/depend

