# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/husarion/catkin_ws/src/mavros/mavros

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/husarion/catkin_ws/build/mavros

# Include any dependencies generated for this target.
include CMakeFiles/libmavros-sensor-orientation-test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/libmavros-sensor-orientation-test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/libmavros-sensor-orientation-test.dir/flags.make

CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o: CMakeFiles/libmavros-sensor-orientation-test.dir/flags.make
CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o: /home/husarion/catkin_ws/src/mavros/mavros/test/test_sensor_orientation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/husarion/catkin_ws/build/mavros/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o -c /home/husarion/catkin_ws/src/mavros/mavros/test/test_sensor_orientation.cpp

CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/husarion/catkin_ws/src/mavros/mavros/test/test_sensor_orientation.cpp > CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.i

CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/husarion/catkin_ws/src/mavros/mavros/test/test_sensor_orientation.cpp -o CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.s

CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o.requires:

.PHONY : CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o.requires

CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o.provides: CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o.requires
	$(MAKE) -f CMakeFiles/libmavros-sensor-orientation-test.dir/build.make CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o.provides.build
.PHONY : CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o.provides

CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o.provides.build: CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o


# Object files for target libmavros-sensor-orientation-test
libmavros__sensor__orientation__test_OBJECTS = \
"CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o"

# External object files for target libmavros-sensor-orientation-test
libmavros__sensor__orientation__test_EXTERNAL_OBJECTS =

/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: CMakeFiles/libmavros-sensor-orientation-test.dir/build.make
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: gtest/googlemock/gtest/libgtest.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /home/husarion/catkin_ws/devel/.private/mavros/lib/libmavros.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /home/husarion/catkin_ws/devel/.private/eigen_conversions/lib/libeigen_conversions.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /home/husarion/catkin_ws/devel/.private/libmavconn/lib/libmavconn.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /opt/ros/melodic/lib/libclass_loader.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/libPocoFoundation.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libdl.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /opt/ros/melodic/lib/libroslib.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /opt/ros/melodic/lib/librospack.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libpython2.7.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_program_options.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libtinyxml2.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /opt/ros/melodic/lib/librosconsole_bridge.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /home/husarion/catkin_ws/devel/.private/tf2_ros/lib/libtf2_ros.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /opt/ros/melodic/lib/libactionlib.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /home/husarion/catkin_ws/devel/.private/message_filters/lib/libmessage_filters.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /home/husarion/catkin_ws/devel/.private/roscpp/lib/libroscpp.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /opt/ros/melodic/lib/librosconsole.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /home/husarion/catkin_ws/devel/.private/xmlrpcpp/lib/libxmlrpcpp.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /home/husarion/catkin_ws/devel/.private/tf2/lib/libtf2.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /opt/ros/melodic/lib/librostime.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /opt/ros/melodic/lib/libcpp_common.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so.0.4
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so.0.4
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: /usr/lib/arm-linux-gnueabihf/libGeographic.so
/home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test: CMakeFiles/libmavros-sensor-orientation-test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/husarion/catkin_ws/build/mavros/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/libmavros-sensor-orientation-test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/libmavros-sensor-orientation-test.dir/build: /home/husarion/catkin_ws/devel/.private/mavros/lib/mavros/libmavros-sensor-orientation-test

.PHONY : CMakeFiles/libmavros-sensor-orientation-test.dir/build

CMakeFiles/libmavros-sensor-orientation-test.dir/requires: CMakeFiles/libmavros-sensor-orientation-test.dir/test/test_sensor_orientation.cpp.o.requires

.PHONY : CMakeFiles/libmavros-sensor-orientation-test.dir/requires

CMakeFiles/libmavros-sensor-orientation-test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/libmavros-sensor-orientation-test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/libmavros-sensor-orientation-test.dir/clean

CMakeFiles/libmavros-sensor-orientation-test.dir/depend:
	cd /home/husarion/catkin_ws/build/mavros && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/husarion/catkin_ws/src/mavros/mavros /home/husarion/catkin_ws/src/mavros/mavros /home/husarion/catkin_ws/build/mavros /home/husarion/catkin_ws/build/mavros /home/husarion/catkin_ws/build/mavros/CMakeFiles/libmavros-sensor-orientation-test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/libmavros-sensor-orientation-test.dir/depend

