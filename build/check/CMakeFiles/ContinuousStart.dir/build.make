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
CMAKE_SOURCE_DIR = /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build

# Utility rule file for ContinuousStart.

# Include the progress variables for this target.
include check/CMakeFiles/ContinuousStart.dir/progress.make

check/CMakeFiles/ContinuousStart:
	cd /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build/check && /usr/bin/ctest -D ContinuousStart

ContinuousStart: check/CMakeFiles/ContinuousStart
ContinuousStart: check/CMakeFiles/ContinuousStart.dir/build.make

.PHONY : ContinuousStart

# Rule to build all files generated by this target.
check/CMakeFiles/ContinuousStart.dir/build: ContinuousStart

.PHONY : check/CMakeFiles/ContinuousStart.dir/build

check/CMakeFiles/ContinuousStart.dir/clean:
	cd /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build/check && $(CMAKE_COMMAND) -P CMakeFiles/ContinuousStart.dir/cmake_clean.cmake
.PHONY : check/CMakeFiles/ContinuousStart.dir/clean

check/CMakeFiles/ContinuousStart.dir/depend:
	cd /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build/check /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build/check/CMakeFiles/ContinuousStart.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : check/CMakeFiles/ContinuousStart.dir/depend

