# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/ashis/catkin_ws/src/quadruped/quadruped_gazebo/plugins

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ashis/catkin_ws/src/quadruped/quadruped_gazebo/plugins/build

# Include any dependencies generated for this target.
include CMakeFiles/robot_cog.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/robot_cog.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/robot_cog.dir/flags.make

CMakeFiles/robot_cog.dir/robot_cog.cc.o: CMakeFiles/robot_cog.dir/flags.make
CMakeFiles/robot_cog.dir/robot_cog.cc.o: ../robot_cog.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ashis/catkin_ws/src/quadruped/quadruped_gazebo/plugins/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/robot_cog.dir/robot_cog.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/robot_cog.dir/robot_cog.cc.o -c /home/ashis/catkin_ws/src/quadruped/quadruped_gazebo/plugins/robot_cog.cc

CMakeFiles/robot_cog.dir/robot_cog.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot_cog.dir/robot_cog.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ashis/catkin_ws/src/quadruped/quadruped_gazebo/plugins/robot_cog.cc > CMakeFiles/robot_cog.dir/robot_cog.cc.i

CMakeFiles/robot_cog.dir/robot_cog.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot_cog.dir/robot_cog.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ashis/catkin_ws/src/quadruped/quadruped_gazebo/plugins/robot_cog.cc -o CMakeFiles/robot_cog.dir/robot_cog.cc.s

CMakeFiles/robot_cog.dir/robot_cog.cc.o.requires:
.PHONY : CMakeFiles/robot_cog.dir/robot_cog.cc.o.requires

CMakeFiles/robot_cog.dir/robot_cog.cc.o.provides: CMakeFiles/robot_cog.dir/robot_cog.cc.o.requires
	$(MAKE) -f CMakeFiles/robot_cog.dir/build.make CMakeFiles/robot_cog.dir/robot_cog.cc.o.provides.build
.PHONY : CMakeFiles/robot_cog.dir/robot_cog.cc.o.provides

CMakeFiles/robot_cog.dir/robot_cog.cc.o.provides.build: CMakeFiles/robot_cog.dir/robot_cog.cc.o

# Object files for target robot_cog
robot_cog_OBJECTS = \
"CMakeFiles/robot_cog.dir/robot_cog.cc.o"

# External object files for target robot_cog
robot_cog_EXTERNAL_OBJECTS =

librobot_cog.so: CMakeFiles/robot_cog.dir/robot_cog.cc.o
librobot_cog.so: CMakeFiles/robot_cog.dir/build.make
librobot_cog.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
librobot_cog.so: CMakeFiles/robot_cog.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library librobot_cog.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robot_cog.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/robot_cog.dir/build: librobot_cog.so
.PHONY : CMakeFiles/robot_cog.dir/build

CMakeFiles/robot_cog.dir/requires: CMakeFiles/robot_cog.dir/robot_cog.cc.o.requires
.PHONY : CMakeFiles/robot_cog.dir/requires

CMakeFiles/robot_cog.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/robot_cog.dir/cmake_clean.cmake
.PHONY : CMakeFiles/robot_cog.dir/clean

CMakeFiles/robot_cog.dir/depend:
	cd /home/ashis/catkin_ws/src/quadruped/quadruped_gazebo/plugins/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ashis/catkin_ws/src/quadruped/quadruped_gazebo/plugins /home/ashis/catkin_ws/src/quadruped/quadruped_gazebo/plugins /home/ashis/catkin_ws/src/quadruped/quadruped_gazebo/plugins/build /home/ashis/catkin_ws/src/quadruped/quadruped_gazebo/plugins/build /home/ashis/catkin_ws/src/quadruped/quadruped_gazebo/plugins/build/CMakeFiles/robot_cog.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/robot_cog.dir/depend
