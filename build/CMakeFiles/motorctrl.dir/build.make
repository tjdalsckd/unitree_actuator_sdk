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
CMAKE_SOURCE_DIR = /home/yuna/workspace/unitree_actuator_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yuna/workspace/unitree_actuator_sdk/build

# Include any dependencies generated for this target.
include CMakeFiles/motorctrl.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/motorctrl.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/motorctrl.dir/flags.make

CMakeFiles/motorctrl.dir/example/main.cpp.o: CMakeFiles/motorctrl.dir/flags.make
CMakeFiles/motorctrl.dir/example/main.cpp.o: ../example/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuna/workspace/unitree_actuator_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/motorctrl.dir/example/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motorctrl.dir/example/main.cpp.o -c /home/yuna/workspace/unitree_actuator_sdk/example/main.cpp

CMakeFiles/motorctrl.dir/example/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motorctrl.dir/example/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuna/workspace/unitree_actuator_sdk/example/main.cpp > CMakeFiles/motorctrl.dir/example/main.cpp.i

CMakeFiles/motorctrl.dir/example/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motorctrl.dir/example/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuna/workspace/unitree_actuator_sdk/example/main.cpp -o CMakeFiles/motorctrl.dir/example/main.cpp.s

# Object files for target motorctrl
motorctrl_OBJECTS = \
"CMakeFiles/motorctrl.dir/example/main.cpp.o"

# External object files for target motorctrl
motorctrl_EXTERNAL_OBJECTS =

motorctrl: CMakeFiles/motorctrl.dir/example/main.cpp.o
motorctrl: CMakeFiles/motorctrl.dir/build.make
motorctrl: CMakeFiles/motorctrl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yuna/workspace/unitree_actuator_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable motorctrl"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/motorctrl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/motorctrl.dir/build: motorctrl

.PHONY : CMakeFiles/motorctrl.dir/build

CMakeFiles/motorctrl.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/motorctrl.dir/cmake_clean.cmake
.PHONY : CMakeFiles/motorctrl.dir/clean

CMakeFiles/motorctrl.dir/depend:
	cd /home/yuna/workspace/unitree_actuator_sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yuna/workspace/unitree_actuator_sdk /home/yuna/workspace/unitree_actuator_sdk /home/yuna/workspace/unitree_actuator_sdk/build /home/yuna/workspace/unitree_actuator_sdk/build /home/yuna/workspace/unitree_actuator_sdk/build/CMakeFiles/motorctrl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/motorctrl.dir/depend

