# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /home/charity/clion/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/charity/clion/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/charity/CLionProjects/openStreetMap

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/charity/CLionProjects/openStreetMap/cmake-build-debug

# Utility rule file for NightlyCoverage.

# Include the progress variables for this target.
include thirdparty/pugixml/CMakeFiles/NightlyCoverage.dir/progress.make

thirdparty/pugixml/CMakeFiles/NightlyCoverage:
	cd /home/charity/CLionProjects/openStreetMap/cmake-build-debug/thirdparty/pugixml && /home/charity/clion/bin/cmake/linux/bin/ctest -D NightlyCoverage

NightlyCoverage: thirdparty/pugixml/CMakeFiles/NightlyCoverage
NightlyCoverage: thirdparty/pugixml/CMakeFiles/NightlyCoverage.dir/build.make

.PHONY : NightlyCoverage

# Rule to build all files generated by this target.
thirdparty/pugixml/CMakeFiles/NightlyCoverage.dir/build: NightlyCoverage

.PHONY : thirdparty/pugixml/CMakeFiles/NightlyCoverage.dir/build

thirdparty/pugixml/CMakeFiles/NightlyCoverage.dir/clean:
	cd /home/charity/CLionProjects/openStreetMap/cmake-build-debug/thirdparty/pugixml && $(CMAKE_COMMAND) -P CMakeFiles/NightlyCoverage.dir/cmake_clean.cmake
.PHONY : thirdparty/pugixml/CMakeFiles/NightlyCoverage.dir/clean

thirdparty/pugixml/CMakeFiles/NightlyCoverage.dir/depend:
	cd /home/charity/CLionProjects/openStreetMap/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/charity/CLionProjects/openStreetMap /home/charity/CLionProjects/openStreetMap/thirdparty/pugixml /home/charity/CLionProjects/openStreetMap/cmake-build-debug /home/charity/CLionProjects/openStreetMap/cmake-build-debug/thirdparty/pugixml /home/charity/CLionProjects/openStreetMap/cmake-build-debug/thirdparty/pugixml/CMakeFiles/NightlyCoverage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : thirdparty/pugixml/CMakeFiles/NightlyCoverage.dir/depend
