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
CMAKE_SOURCE_DIR = /home/rgblab/ViewLink/ViewLink_Python

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rgblab/ViewLink/ViewLink_Python/build

# Include any dependencies generated for this target.
include src/CMakeFiles/vlkpy.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/CMakeFiles/vlkpy.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/vlkpy.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/vlkpy.dir/flags.make

src/CMakeFiles/vlkpy.dir/src.cpp.o: src/CMakeFiles/vlkpy.dir/flags.make
src/CMakeFiles/vlkpy.dir/src.cpp.o: ../src/src.cpp
src/CMakeFiles/vlkpy.dir/src.cpp.o: src/CMakeFiles/vlkpy.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rgblab/ViewLink/ViewLink_Python/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/vlkpy.dir/src.cpp.o"
	cd /home/rgblab/ViewLink/ViewLink_Python/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/vlkpy.dir/src.cpp.o -MF CMakeFiles/vlkpy.dir/src.cpp.o.d -o CMakeFiles/vlkpy.dir/src.cpp.o -c /home/rgblab/ViewLink/ViewLink_Python/src/src.cpp

src/CMakeFiles/vlkpy.dir/src.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vlkpy.dir/src.cpp.i"
	cd /home/rgblab/ViewLink/ViewLink_Python/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rgblab/ViewLink/ViewLink_Python/src/src.cpp > CMakeFiles/vlkpy.dir/src.cpp.i

src/CMakeFiles/vlkpy.dir/src.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vlkpy.dir/src.cpp.s"
	cd /home/rgblab/ViewLink/ViewLink_Python/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rgblab/ViewLink/ViewLink_Python/src/src.cpp -o CMakeFiles/vlkpy.dir/src.cpp.s

# Object files for target vlkpy
vlkpy_OBJECTS = \
"CMakeFiles/vlkpy.dir/src.cpp.o"

# External object files for target vlkpy
vlkpy_EXTERNAL_OBJECTS =

src/vlkpy.cpython-310-aarch64-linux-gnu.so: src/CMakeFiles/vlkpy.dir/src.cpp.o
src/vlkpy.cpython-310-aarch64-linux-gnu.so: src/CMakeFiles/vlkpy.dir/build.make
src/vlkpy.cpython-310-aarch64-linux-gnu.so: src/CMakeFiles/vlkpy.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rgblab/ViewLink/ViewLink_Python/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared module vlkpy.cpython-310-aarch64-linux-gnu.so"
	cd /home/rgblab/ViewLink/ViewLink_Python/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vlkpy.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/vlkpy.dir/build: src/vlkpy.cpython-310-aarch64-linux-gnu.so
.PHONY : src/CMakeFiles/vlkpy.dir/build

src/CMakeFiles/vlkpy.dir/clean:
	cd /home/rgblab/ViewLink/ViewLink_Python/build/src && $(CMAKE_COMMAND) -P CMakeFiles/vlkpy.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/vlkpy.dir/clean

src/CMakeFiles/vlkpy.dir/depend:
	cd /home/rgblab/ViewLink/ViewLink_Python/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rgblab/ViewLink/ViewLink_Python /home/rgblab/ViewLink/ViewLink_Python/src /home/rgblab/ViewLink/ViewLink_Python/build /home/rgblab/ViewLink/ViewLink_Python/build/src /home/rgblab/ViewLink/ViewLink_Python/build/src/CMakeFiles/vlkpy.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/vlkpy.dir/depend

