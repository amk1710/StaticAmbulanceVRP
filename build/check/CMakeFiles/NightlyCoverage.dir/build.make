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

# Utility rule file for NightlyCoverage.

# Include the progress variables for this target.
include check/CMakeFiles/NightlyCoverage.dir/progress.make

check/CMakeFiles/NightlyCoverage:
	cd /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build/check && /usr/bin/ctest -D NightlyCoverage

NightlyCoverage: check/CMakeFiles/NightlyCoverage
NightlyCoverage: check/CMakeFiles/NightlyCoverage.dir/build.make

.PHONY : NightlyCoverage

# Rule to build all files generated by this target.
check/CMakeFiles/NightlyCoverage.dir/build: NightlyCoverage

.PHONY : check/CMakeFiles/NightlyCoverage.dir/build

check/CMakeFiles/NightlyCoverage.dir/clean:
	cd /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build/check && $(CMAKE_COMMAND) -P CMakeFiles/NightlyCoverage.dir/cmake_clean.cmake
.PHONY : check/CMakeFiles/NightlyCoverage.dir/clean

check/CMakeFiles/NightlyCoverage.dir/depend:
	cd /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/check /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build/check /mnt/d/Documents/NaoPSR/MasterThesis/StaticAmbulanceVRP/build/check/CMakeFiles/NightlyCoverage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : check/CMakeFiles/NightlyCoverage.dir/depend

