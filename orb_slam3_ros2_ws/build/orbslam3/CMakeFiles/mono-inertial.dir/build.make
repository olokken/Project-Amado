# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ole/Dev/project-amado/orb_slam3_ros2_ws/build/orbslam3

# Include any dependencies generated for this target.
include CMakeFiles/mono-inertial.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/mono-inertial.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/mono-inertial.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mono-inertial.dir/flags.make

CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.o: CMakeFiles/mono-inertial.dir/flags.make
CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.o: /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/src/mono-inertial/mono-inertial.cpp
CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.o: CMakeFiles/mono-inertial.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ole/Dev/project-amado/orb_slam3_ros2_ws/build/orbslam3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.o -MF CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.o.d -o CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.o -c /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/src/mono-inertial/mono-inertial.cpp

CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/src/mono-inertial/mono-inertial.cpp > CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.i

CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/src/mono-inertial/mono-inertial.cpp -o CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.s

CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.o: CMakeFiles/mono-inertial.dir/flags.make
CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.o: /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/src/mono-inertial/mono-inertial-slam-node.cpp
CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.o: CMakeFiles/mono-inertial.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ole/Dev/project-amado/orb_slam3_ros2_ws/build/orbslam3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.o -MF CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.o.d -o CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.o -c /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/src/mono-inertial/mono-inertial-slam-node.cpp

CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/src/mono-inertial/mono-inertial-slam-node.cpp > CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.i

CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/src/mono-inertial/mono-inertial-slam-node.cpp -o CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.s

# Object files for target mono-inertial
mono__inertial_OBJECTS = \
"CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.o" \
"CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.o"

# External object files for target mono-inertial
mono__inertial_EXTERNAL_OBJECTS =

mono-inertial: CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial.cpp.o
mono-inertial: CMakeFiles/mono-inertial.dir/src/mono-inertial/mono-inertial-slam-node.cpp.o
mono-inertial: CMakeFiles/mono-inertial.dir/build.make
mono-inertial: /opt/ros/humble/lib/libcv_bridge.so
mono-inertial: /opt/ros/humble/lib/libstatic_transform_broadcaster_node.so
mono-inertial: /home/ole/Dev/project-amado/orb-slam3-stereo-fixed-master/lib/libORB_SLAM3.so
mono-inertial: /home/ole/Dev/project-amado/orb-slam3-stereo-fixed-master/Thirdparty/DBoW2/lib/libDBoW2.so
mono-inertial: /home/ole/Dev/project-amado/orb-slam3-stereo-fixed-master/Thirdparty/g2o/lib/libg2o.so
mono-inertial: /usr/local/lib/libpango_glgeometry.so
mono-inertial: /usr/local/lib/libpango_python.so
mono-inertial: /usr/local/lib/libpango_scene.so
mono-inertial: /usr/local/lib/libpango_tools.so
mono-inertial: /usr/local/lib/libpango_video.so
mono-inertial: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_c.so
mono-inertial: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so
mono-inertial: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
mono-inertial: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
mono-inertial: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_py.so
mono-inertial: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_c.so
mono-inertial: /opt/ros/humble/lib/libsensor_msgs__rosidl_generator_c.so
mono-inertial: /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_cpp.so
mono-inertial: /usr/local/lib/libopencv_imgcodecs.so.3.4.20
mono-inertial: /usr/local/lib/libopencv_imgproc.so.3.4.20
mono-inertial: /usr/local/lib/libopencv_core.so.3.4.20
mono-inertial: /opt/ros/humble/lib/libtf2_ros.so
mono-inertial: /opt/ros/humble/lib/libtf2.so
mono-inertial: /opt/ros/humble/lib/libmessage_filters.so
mono-inertial: /opt/ros/humble/lib/librclcpp_action.so
mono-inertial: /opt/ros/humble/lib/librclcpp.so
mono-inertial: /opt/ros/humble/lib/liblibstatistics_collector.so
mono-inertial: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
mono-inertial: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
mono-inertial: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
mono-inertial: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
mono-inertial: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
mono-inertial: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_py.so
mono-inertial: /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_c.so
mono-inertial: /opt/ros/humble/lib/librosgraph_msgs__rosidl_generator_c.so
mono-inertial: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
mono-inertial: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
mono-inertial: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
mono-inertial: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
mono-inertial: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
mono-inertial: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_py.so
mono-inertial: /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_c.so
mono-inertial: /opt/ros/humble/lib/libstatistics_msgs__rosidl_generator_c.so
mono-inertial: /opt/ros/humble/lib/librcl_action.so
mono-inertial: /opt/ros/humble/lib/librcl.so
mono-inertial: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
mono-inertial: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
mono-inertial: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
mono-inertial: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
mono-inertial: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_cpp.so
mono-inertial: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_py.so
mono-inertial: /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_c.so
mono-inertial: /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_c.so
mono-inertial: /opt/ros/humble/lib/librcl_yaml_param_parser.so
mono-inertial: /opt/ros/humble/lib/libyaml.so
mono-inertial: /opt/ros/humble/lib/libtracetools.so
mono-inertial: /opt/ros/humble/lib/librmw_implementation.so
mono-inertial: /opt/ros/humble/lib/libament_index_cpp.so
mono-inertial: /opt/ros/humble/lib/librcl_logging_spdlog.so
mono-inertial: /opt/ros/humble/lib/librcl_logging_interface.so
mono-inertial: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_fastrtps_c.so
mono-inertial: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
mono-inertial: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
mono-inertial: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_fastrtps_c.so
mono-inertial: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
mono-inertial: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_fastrtps_c.so
mono-inertial: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_c.so
mono-inertial: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_introspection_c.so
mono-inertial: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mono-inertial: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
mono-inertial: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_introspection_c.so
mono-inertial: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
mono-inertial: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_c.so
mono-inertial: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_fastrtps_cpp.so
mono-inertial: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
mono-inertial: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
mono-inertial: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_fastrtps_cpp.so
mono-inertial: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
mono-inertial: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_fastrtps_cpp.so
mono-inertial: /opt/ros/humble/lib/librosidl_typesupport_fastrtps_cpp.so
mono-inertial: /opt/ros/humble/lib/libfastcdr.so.1.0.24
mono-inertial: /opt/ros/humble/lib/librmw.so
mono-inertial: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_introspection_cpp.so
mono-inertial: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mono-inertial: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
mono-inertial: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_introspection_cpp.so
mono-inertial: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
mono-inertial: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_cpp.so
mono-inertial: /opt/ros/humble/lib/librosidl_typesupport_introspection_cpp.so
mono-inertial: /opt/ros/humble/lib/librosidl_typesupport_introspection_c.so
mono-inertial: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_cpp.so
mono-inertial: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mono-inertial: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
mono-inertial: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_cpp.so
mono-inertial: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
mono-inertial: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_cpp.so
mono-inertial: /opt/ros/humble/lib/librosidl_typesupport_cpp.so
mono-inertial: /opt/ros/humble/lib/libtf2_msgs__rosidl_generator_py.so
mono-inertial: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_py.so
mono-inertial: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_py.so
mono-inertial: /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_c.so
mono-inertial: /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_c.so
mono-inertial: /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_c.so
mono-inertial: /opt/ros/humble/lib/libtf2_msgs__rosidl_generator_c.so
mono-inertial: /opt/ros/humble/lib/libgeometry_msgs__rosidl_generator_c.so
mono-inertial: /opt/ros/humble/lib/libstd_msgs__rosidl_generator_c.so
mono-inertial: /opt/ros/humble/lib/libaction_msgs__rosidl_generator_py.so
mono-inertial: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_py.so
mono-inertial: /opt/ros/humble/lib/libaction_msgs__rosidl_typesupport_c.so
mono-inertial: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
mono-inertial: /opt/ros/humble/lib/libaction_msgs__rosidl_generator_c.so
mono-inertial: /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so
mono-inertial: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_generator_py.so
mono-inertial: /usr/lib/x86_64-linux-gnu/libpython3.10.so
mono-inertial: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_typesupport_c.so
mono-inertial: /opt/ros/humble/lib/librosidl_typesupport_c.so
mono-inertial: /opt/ros/humble/lib/librcpputils.so
mono-inertial: /opt/ros/humble/lib/libunique_identifier_msgs__rosidl_generator_c.so
mono-inertial: /opt/ros/humble/lib/librosidl_runtime_c.so
mono-inertial: /opt/ros/humble/lib/librcutils.so
mono-inertial: /usr/local/lib/libpango_geometry.so
mono-inertial: /usr/local/lib/libtinyobj.so
mono-inertial: /usr/local/lib/libpango_plot.so
mono-inertial: /usr/local/lib/libpango_display.so
mono-inertial: /usr/local/lib/libpango_vars.so
mono-inertial: /usr/local/lib/libpango_windowing.so
mono-inertial: /usr/local/lib/libpango_opengl.so
mono-inertial: /usr/lib/x86_64-linux-gnu/libGLEW.so
mono-inertial: /usr/lib/x86_64-linux-gnu/libOpenGL.so
mono-inertial: /usr/lib/x86_64-linux-gnu/libGLX.so
mono-inertial: /usr/lib/x86_64-linux-gnu/libGLU.so
mono-inertial: /usr/local/lib/libpango_image.so
mono-inertial: /usr/local/lib/libpango_packetstream.so
mono-inertial: /usr/local/lib/libpango_core.so
mono-inertial: CMakeFiles/mono-inertial.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ole/Dev/project-amado/orb_slam3_ros2_ws/build/orbslam3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable mono-inertial"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mono-inertial.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mono-inertial.dir/build: mono-inertial
.PHONY : CMakeFiles/mono-inertial.dir/build

CMakeFiles/mono-inertial.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mono-inertial.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mono-inertial.dir/clean

CMakeFiles/mono-inertial.dir/depend:
	cd /home/ole/Dev/project-amado/orb_slam3_ros2_ws/build/orbslam3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main /home/ole/Dev/project-amado/orb_slam3_ros2_ws/build/orbslam3 /home/ole/Dev/project-amado/orb_slam3_ros2_ws/build/orbslam3 /home/ole/Dev/project-amado/orb_slam3_ros2_ws/build/orbslam3/CMakeFiles/mono-inertial.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mono-inertial.dir/depend
