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
CMAKE_SOURCE_DIR = /media/flora/myros/new_re2_ws/src/octocan

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/flora/myros/new_re2_ws/build/octocan

# Include any dependencies generated for this target.
include CMakeFiles/octocan.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/octocan.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/octocan.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/octocan.dir/flags.make

CMakeFiles/octocan.dir/src/skintalk.c.o: CMakeFiles/octocan.dir/flags.make
CMakeFiles/octocan.dir/src/skintalk.c.o: /media/flora/myros/new_re2_ws/src/octocan/src/skintalk.c
CMakeFiles/octocan.dir/src/skintalk.c.o: CMakeFiles/octocan.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/flora/myros/new_re2_ws/build/octocan/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/octocan.dir/src/skintalk.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/octocan.dir/src/skintalk.c.o -MF CMakeFiles/octocan.dir/src/skintalk.c.o.d -o CMakeFiles/octocan.dir/src/skintalk.c.o -c /media/flora/myros/new_re2_ws/src/octocan/src/skintalk.c

CMakeFiles/octocan.dir/src/skintalk.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/octocan.dir/src/skintalk.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /media/flora/myros/new_re2_ws/src/octocan/src/skintalk.c > CMakeFiles/octocan.dir/src/skintalk.c.i

CMakeFiles/octocan.dir/src/skintalk.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/octocan.dir/src/skintalk.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /media/flora/myros/new_re2_ws/src/octocan/src/skintalk.c -o CMakeFiles/octocan.dir/src/skintalk.c.s

CMakeFiles/octocan.dir/src/profile.c.o: CMakeFiles/octocan.dir/flags.make
CMakeFiles/octocan.dir/src/profile.c.o: /media/flora/myros/new_re2_ws/src/octocan/src/profile.c
CMakeFiles/octocan.dir/src/profile.c.o: CMakeFiles/octocan.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/flora/myros/new_re2_ws/build/octocan/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/octocan.dir/src/profile.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/octocan.dir/src/profile.c.o -MF CMakeFiles/octocan.dir/src/profile.c.o.d -o CMakeFiles/octocan.dir/src/profile.c.o -c /media/flora/myros/new_re2_ws/src/octocan/src/profile.c

CMakeFiles/octocan.dir/src/profile.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/octocan.dir/src/profile.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /media/flora/myros/new_re2_ws/src/octocan/src/profile.c > CMakeFiles/octocan.dir/src/profile.c.i

CMakeFiles/octocan.dir/src/profile.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/octocan.dir/src/profile.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /media/flora/myros/new_re2_ws/src/octocan/src/profile.c -o CMakeFiles/octocan.dir/src/profile.c.s

CMakeFiles/octocan.dir/src/layout.c.o: CMakeFiles/octocan.dir/flags.make
CMakeFiles/octocan.dir/src/layout.c.o: /media/flora/myros/new_re2_ws/src/octocan/src/layout.c
CMakeFiles/octocan.dir/src/layout.c.o: CMakeFiles/octocan.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/flora/myros/new_re2_ws/build/octocan/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/octocan.dir/src/layout.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/octocan.dir/src/layout.c.o -MF CMakeFiles/octocan.dir/src/layout.c.o.d -o CMakeFiles/octocan.dir/src/layout.c.o -c /media/flora/myros/new_re2_ws/src/octocan/src/layout.c

CMakeFiles/octocan.dir/src/layout.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/octocan.dir/src/layout.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /media/flora/myros/new_re2_ws/src/octocan/src/layout.c > CMakeFiles/octocan.dir/src/layout.c.i

CMakeFiles/octocan.dir/src/layout.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/octocan.dir/src/layout.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /media/flora/myros/new_re2_ws/src/octocan/src/layout.c -o CMakeFiles/octocan.dir/src/layout.c.s

# Object files for target octocan
octocan_OBJECTS = \
"CMakeFiles/octocan.dir/src/skintalk.c.o" \
"CMakeFiles/octocan.dir/src/profile.c.o" \
"CMakeFiles/octocan.dir/src/layout.c.o"

# External object files for target octocan
octocan_EXTERNAL_OBJECTS =

liboctocan.a: CMakeFiles/octocan.dir/src/skintalk.c.o
liboctocan.a: CMakeFiles/octocan.dir/src/profile.c.o
liboctocan.a: CMakeFiles/octocan.dir/src/layout.c.o
liboctocan.a: CMakeFiles/octocan.dir/build.make
liboctocan.a: CMakeFiles/octocan.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/flora/myros/new_re2_ws/build/octocan/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking C static library liboctocan.a"
	$(CMAKE_COMMAND) -P CMakeFiles/octocan.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/octocan.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/octocan.dir/build: liboctocan.a
.PHONY : CMakeFiles/octocan.dir/build

CMakeFiles/octocan.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/octocan.dir/cmake_clean.cmake
.PHONY : CMakeFiles/octocan.dir/clean

CMakeFiles/octocan.dir/depend:
	cd /media/flora/myros/new_re2_ws/build/octocan && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/flora/myros/new_re2_ws/src/octocan /media/flora/myros/new_re2_ws/src/octocan /media/flora/myros/new_re2_ws/build/octocan /media/flora/myros/new_re2_ws/build/octocan /media/flora/myros/new_re2_ws/build/octocan/CMakeFiles/octocan.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/octocan.dir/depend
