# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_SOURCE_DIR = /home/lici/apps/cs225a

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lici/apps/cs225a/hw2

# Include any dependencies generated for this target.
include lecture2-demo/CMakeFiles/controller.dir/depend.make

# Include the progress variables for this target.
include lecture2-demo/CMakeFiles/controller.dir/progress.make

# Include the compile flags for this target's objects.
include lecture2-demo/CMakeFiles/controller.dir/flags.make

lecture2-demo/CMakeFiles/controller.dir/controller.cpp.o: lecture2-demo/CMakeFiles/controller.dir/flags.make
lecture2-demo/CMakeFiles/controller.dir/controller.cpp.o: ../lecture2-demo/controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lici/apps/cs225a/hw2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lecture2-demo/CMakeFiles/controller.dir/controller.cpp.o"
	cd /home/lici/apps/cs225a/hw2/lecture2-demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller.dir/controller.cpp.o -c /home/lici/apps/cs225a/lecture2-demo/controller.cpp

lecture2-demo/CMakeFiles/controller.dir/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller.dir/controller.cpp.i"
	cd /home/lici/apps/cs225a/hw2/lecture2-demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lici/apps/cs225a/lecture2-demo/controller.cpp > CMakeFiles/controller.dir/controller.cpp.i

lecture2-demo/CMakeFiles/controller.dir/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller.dir/controller.cpp.s"
	cd /home/lici/apps/cs225a/hw2/lecture2-demo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lici/apps/cs225a/lecture2-demo/controller.cpp -o CMakeFiles/controller.dir/controller.cpp.s

# Object files for target controller
controller_OBJECTS = \
"CMakeFiles/controller.dir/controller.cpp.o"

# External object files for target controller
controller_EXTERNAL_OBJECTS =

../bin/lecture2-demo/controller: lecture2-demo/CMakeFiles/controller.dir/controller.cpp.o
../bin/lecture2-demo/controller: lecture2-demo/CMakeFiles/controller.dir/build.make
../bin/lecture2-demo/controller: /home/lici/core/sai2-common/build/libsai2-common.a
../bin/lecture2-demo/controller: /home/lici/core/chai3d/build/libchai3d.a
../bin/lecture2-demo/controller: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/lecture2-demo/controller: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/lecture2-demo/controller: /home/lici/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/lecture2-demo/controller: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
../bin/lecture2-demo/controller: /home/lici/core/sai2-simulation-master/build/libsai2-simulation.a
../bin/lecture2-demo/controller: /home/lici/core/sai2-model/build/libsai2-model.a
../bin/lecture2-demo/controller: /home/lici/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/lecture2-demo/controller: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
../bin/lecture2-demo/controller: /home/lici/core/sai2-model/rbdl/build/librbdl.so
../bin/lecture2-demo/controller: /home/lici/core/sai2-graphics/build/libsai2-graphics.a
../bin/lecture2-demo/controller: /home/lici/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/lecture2-demo/controller: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
../bin/lecture2-demo/controller: /home/lici/core/chai3d/build/libchai3d.a
../bin/lecture2-demo/controller: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/lecture2-demo/controller: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/lecture2-demo/controller: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/lecture2-demo/controller: /usr/lib/x86_64-linux-gnu/libglfw.so
../bin/lecture2-demo/controller: /home/lici/core/sai2-model/rbdl/build/librbdl.so
../bin/lecture2-demo/controller: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/lecture2-demo/controller: /usr/lib/x86_64-linux-gnu/libglfw.so
../bin/lecture2-demo/controller: lecture2-demo/CMakeFiles/controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lici/apps/cs225a/hw2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/lecture2-demo/controller"
	cd /home/lici/apps/cs225a/hw2/lecture2-demo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lecture2-demo/CMakeFiles/controller.dir/build: ../bin/lecture2-demo/controller

.PHONY : lecture2-demo/CMakeFiles/controller.dir/build

lecture2-demo/CMakeFiles/controller.dir/clean:
	cd /home/lici/apps/cs225a/hw2/lecture2-demo && $(CMAKE_COMMAND) -P CMakeFiles/controller.dir/cmake_clean.cmake
.PHONY : lecture2-demo/CMakeFiles/controller.dir/clean

lecture2-demo/CMakeFiles/controller.dir/depend:
	cd /home/lici/apps/cs225a/hw2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lici/apps/cs225a /home/lici/apps/cs225a/lecture2-demo /home/lici/apps/cs225a/hw2 /home/lici/apps/cs225a/hw2/lecture2-demo /home/lici/apps/cs225a/hw2/lecture2-demo/CMakeFiles/controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lecture2-demo/CMakeFiles/controller.dir/depend

