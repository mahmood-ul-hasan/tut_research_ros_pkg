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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /media/aisl2/aisl_data/catkin_ws/src/sky_detector

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/aisl2/aisl_data/catkin_ws/src/sky_detector/build

# Include any dependencies generated for this target.
include CMakeFiles/detector.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/detector.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/detector.dir/flags.make

CMakeFiles/detector.dir/main_test.cpp.o: CMakeFiles/detector.dir/flags.make
CMakeFiles/detector.dir/main_test.cpp.o: ../main_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/aisl2/aisl_data/catkin_ws/src/sky_detector/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/detector.dir/main_test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detector.dir/main_test.cpp.o -c /media/aisl2/aisl_data/catkin_ws/src/sky_detector/main_test.cpp

CMakeFiles/detector.dir/main_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detector.dir/main_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/aisl2/aisl_data/catkin_ws/src/sky_detector/main_test.cpp > CMakeFiles/detector.dir/main_test.cpp.i

CMakeFiles/detector.dir/main_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detector.dir/main_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/aisl2/aisl_data/catkin_ws/src/sky_detector/main_test.cpp -o CMakeFiles/detector.dir/main_test.cpp.s

CMakeFiles/detector.dir/sky_detector/imageSkyDetector.cpp.o: CMakeFiles/detector.dir/flags.make
CMakeFiles/detector.dir/sky_detector/imageSkyDetector.cpp.o: ../sky_detector/imageSkyDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/aisl2/aisl_data/catkin_ws/src/sky_detector/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/detector.dir/sky_detector/imageSkyDetector.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detector.dir/sky_detector/imageSkyDetector.cpp.o -c /media/aisl2/aisl_data/catkin_ws/src/sky_detector/sky_detector/imageSkyDetector.cpp

CMakeFiles/detector.dir/sky_detector/imageSkyDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detector.dir/sky_detector/imageSkyDetector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/aisl2/aisl_data/catkin_ws/src/sky_detector/sky_detector/imageSkyDetector.cpp > CMakeFiles/detector.dir/sky_detector/imageSkyDetector.cpp.i

CMakeFiles/detector.dir/sky_detector/imageSkyDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detector.dir/sky_detector/imageSkyDetector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/aisl2/aisl_data/catkin_ws/src/sky_detector/sky_detector/imageSkyDetector.cpp -o CMakeFiles/detector.dir/sky_detector/imageSkyDetector.cpp.s

CMakeFiles/detector.dir/file_processor/file_system_processor.cpp.o: CMakeFiles/detector.dir/flags.make
CMakeFiles/detector.dir/file_processor/file_system_processor.cpp.o: ../file_processor/file_system_processor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/aisl2/aisl_data/catkin_ws/src/sky_detector/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/detector.dir/file_processor/file_system_processor.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detector.dir/file_processor/file_system_processor.cpp.o -c /media/aisl2/aisl_data/catkin_ws/src/sky_detector/file_processor/file_system_processor.cpp

CMakeFiles/detector.dir/file_processor/file_system_processor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detector.dir/file_processor/file_system_processor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/aisl2/aisl_data/catkin_ws/src/sky_detector/file_processor/file_system_processor.cpp > CMakeFiles/detector.dir/file_processor/file_system_processor.cpp.i

CMakeFiles/detector.dir/file_processor/file_system_processor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detector.dir/file_processor/file_system_processor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/aisl2/aisl_data/catkin_ws/src/sky_detector/file_processor/file_system_processor.cpp -o CMakeFiles/detector.dir/file_processor/file_system_processor.cpp.s

# Object files for target detector
detector_OBJECTS = \
"CMakeFiles/detector.dir/main_test.cpp.o" \
"CMakeFiles/detector.dir/sky_detector/imageSkyDetector.cpp.o" \
"CMakeFiles/detector.dir/file_processor/file_system_processor.cpp.o"

# External object files for target detector
detector_EXTERNAL_OBJECTS =

detector: CMakeFiles/detector.dir/main_test.cpp.o
detector: CMakeFiles/detector.dir/sky_detector/imageSkyDetector.cpp.o
detector: CMakeFiles/detector.dir/file_processor/file_system_processor.cpp.o
detector: CMakeFiles/detector.dir/build.make
detector: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
detector: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
detector: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
detector: CMakeFiles/detector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/aisl2/aisl_data/catkin_ws/src/sky_detector/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable detector"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/detector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/detector.dir/build: detector

.PHONY : CMakeFiles/detector.dir/build

CMakeFiles/detector.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/detector.dir/cmake_clean.cmake
.PHONY : CMakeFiles/detector.dir/clean

CMakeFiles/detector.dir/depend:
	cd /media/aisl2/aisl_data/catkin_ws/src/sky_detector/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/aisl2/aisl_data/catkin_ws/src/sky_detector /media/aisl2/aisl_data/catkin_ws/src/sky_detector /media/aisl2/aisl_data/catkin_ws/src/sky_detector/build /media/aisl2/aisl_data/catkin_ws/src/sky_detector/build /media/aisl2/aisl_data/catkin_ws/src/sky_detector/build/CMakeFiles/detector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/detector.dir/depend

