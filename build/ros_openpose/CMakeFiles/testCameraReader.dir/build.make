# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/student/seher/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/seher/build

# Include any dependencies generated for this target.
include ros_openpose/CMakeFiles/testCameraReader.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include ros_openpose/CMakeFiles/testCameraReader.dir/compiler_depend.make

# Include the progress variables for this target.
include ros_openpose/CMakeFiles/testCameraReader.dir/progress.make

# Include the compile flags for this target's objects.
include ros_openpose/CMakeFiles/testCameraReader.dir/flags.make

ros_openpose/CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.o: ros_openpose/CMakeFiles/testCameraReader.dir/flags.make
ros_openpose/CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.o: /home/student/seher/src/ros_openpose/src/testCameraReader.cpp
ros_openpose/CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.o: ros_openpose/CMakeFiles/testCameraReader.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/seher/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ros_openpose/CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.o"
	cd /home/student/seher/build/ros_openpose && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ros_openpose/CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.o -MF CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.o.d -o CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.o -c /home/student/seher/src/ros_openpose/src/testCameraReader.cpp

ros_openpose/CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.i"
	cd /home/student/seher/build/ros_openpose && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/seher/src/ros_openpose/src/testCameraReader.cpp > CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.i

ros_openpose/CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.s"
	cd /home/student/seher/build/ros_openpose && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/seher/src/ros_openpose/src/testCameraReader.cpp -o CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.s

ros_openpose/CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.o: ros_openpose/CMakeFiles/testCameraReader.dir/flags.make
ros_openpose/CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.o: /home/student/seher/src/ros_openpose/src/cameraReader.cpp
ros_openpose/CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.o: ros_openpose/CMakeFiles/testCameraReader.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/seher/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object ros_openpose/CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.o"
	cd /home/student/seher/build/ros_openpose && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ros_openpose/CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.o -MF CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.o.d -o CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.o -c /home/student/seher/src/ros_openpose/src/cameraReader.cpp

ros_openpose/CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.i"
	cd /home/student/seher/build/ros_openpose && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/seher/src/ros_openpose/src/cameraReader.cpp > CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.i

ros_openpose/CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.s"
	cd /home/student/seher/build/ros_openpose && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/seher/src/ros_openpose/src/cameraReader.cpp -o CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.s

# Object files for target testCameraReader
testCameraReader_OBJECTS = \
"CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.o" \
"CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.o"

# External object files for target testCameraReader
testCameraReader_EXTERNAL_OBJECTS =

/home/student/seher/devel/lib/ros_openpose/testCameraReader: ros_openpose/CMakeFiles/testCameraReader.dir/src/testCameraReader.cpp.o
/home/student/seher/devel/lib/ros_openpose/testCameraReader: ros_openpose/CMakeFiles/testCameraReader.dir/src/cameraReader.cpp.o
/home/student/seher/devel/lib/ros_openpose/testCameraReader: ros_openpose/CMakeFiles/testCameraReader.dir/build.make
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /home/student/seher/src/openpose/CMakeFiles/Export/lib/libopenpose.so.1.6.0
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libgflags.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libglog.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_stitching.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_superres.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_videostab.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_aruco.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_bgsegm.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_bioinspired.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_ccalib.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_dpm.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_face.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_freetype.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_fuzzy.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_hdf.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_hfs.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_img_hash.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_line_descriptor.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_optflow.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_reg.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_rgbd.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_saliency.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_sfm.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_stereo.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_structured_light.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_surface_matching.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_tracking.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_xfeatures2d.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_ximgproc.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_xobjdetect.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_xphoto.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/libcv_bridge.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/libimage_transport.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/libclass_loader.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/libPocoFoundation.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libdl.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/libroslib.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/librospack.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/libmessage_filters.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/libroscpp.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/librosconsole.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/librostime.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /opt/ros/melodic/lib/libcpp_common.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_viz.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_datasets.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_plot.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_text.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_dnn.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_ml.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_shape.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_video.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_objdetect.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_calib3d.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_features2d.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_flann.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_highgui.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_videoio.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_imgcodecs.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_photo.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_imgproc.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/lib/libopencv_core.so.3.4.5
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/local/cuda-10.1/lib64/libcudart_static.a
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/librt.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /home/student/seher/src/openpose/caffe/lib/libcaffe.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: /usr/lib/x86_64-linux-gnu/libglog.so
/home/student/seher/devel/lib/ros_openpose/testCameraReader: ros_openpose/CMakeFiles/testCameraReader.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/student/seher/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/student/seher/devel/lib/ros_openpose/testCameraReader"
	cd /home/student/seher/build/ros_openpose && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testCameraReader.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ros_openpose/CMakeFiles/testCameraReader.dir/build: /home/student/seher/devel/lib/ros_openpose/testCameraReader
.PHONY : ros_openpose/CMakeFiles/testCameraReader.dir/build

ros_openpose/CMakeFiles/testCameraReader.dir/clean:
	cd /home/student/seher/build/ros_openpose && $(CMAKE_COMMAND) -P CMakeFiles/testCameraReader.dir/cmake_clean.cmake
.PHONY : ros_openpose/CMakeFiles/testCameraReader.dir/clean

ros_openpose/CMakeFiles/testCameraReader.dir/depend:
	cd /home/student/seher/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/seher/src /home/student/seher/src/ros_openpose /home/student/seher/build /home/student/seher/build/ros_openpose /home/student/seher/build/ros_openpose/CMakeFiles/testCameraReader.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ros_openpose/CMakeFiles/testCameraReader.dir/depend

