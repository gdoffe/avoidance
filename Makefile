# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:

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
CMAKE_SOURCE_DIR = /home/gdo/Developpement/Informatique/Personnel/cogip/avoidance

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gdo/Developpement/Informatique/Personnel/cogip/avoidance

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --cyan "Running CMake cache editor..."
	/usr/bin/cmake-gui -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake --regenerate-during-build -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/gdo/Developpement/Informatique/Personnel/cogip/avoidance/CMakeFiles /home/gdo/Developpement/Informatique/Personnel/cogip/avoidance//CMakeFiles/progress.marks
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/gdo/Developpement/Informatique/Personnel/cogip/avoidance/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named avoidance_lib

# Build rule for target.
avoidance_lib: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 avoidance_lib
.PHONY : avoidance_lib

# fast build rule for target.
avoidance_lib/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/build
.PHONY : avoidance_lib/fast

#=============================================================================
# Target rules for targets named test_avoidance

# Build rule for target.
test_avoidance: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 test_avoidance
.PHONY : test_avoidance

# fast build rule for target.
test_avoidance/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/test_avoidance.dir/build.make CMakeFiles/test_avoidance.dir/build
.PHONY : test_avoidance/fast

#=============================================================================
# Target rules for targets named pyavoidance

# Build rule for target.
pyavoidance: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 pyavoidance
.PHONY : pyavoidance

# fast build rule for target.
pyavoidance/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/pyavoidance.dir/build.make CMakeFiles/pyavoidance.dir/build
.PHONY : pyavoidance/fast

applications/tests/avoidance/main.o: applications/tests/avoidance/main.cpp.o
.PHONY : applications/tests/avoidance/main.o

# target to build an object file
applications/tests/avoidance/main.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/test_avoidance.dir/build.make CMakeFiles/test_avoidance.dir/applications/tests/avoidance/main.cpp.o
.PHONY : applications/tests/avoidance/main.cpp.o

applications/tests/avoidance/main.i: applications/tests/avoidance/main.cpp.i
.PHONY : applications/tests/avoidance/main.i

# target to preprocess a source file
applications/tests/avoidance/main.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/test_avoidance.dir/build.make CMakeFiles/test_avoidance.dir/applications/tests/avoidance/main.cpp.i
.PHONY : applications/tests/avoidance/main.cpp.i

applications/tests/avoidance/main.s: applications/tests/avoidance/main.cpp.s
.PHONY : applications/tests/avoidance/main.s

# target to generate assembly for a file
applications/tests/avoidance/main.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/test_avoidance.dir/build.make CMakeFiles/test_avoidance.dir/applications/tests/avoidance/main.cpp.s
.PHONY : applications/tests/avoidance/main.cpp.s

applications/tests/avoidance/pyavoidance.o: applications/tests/avoidance/pyavoidance.cxx.o
.PHONY : applications/tests/avoidance/pyavoidance.o

# target to build an object file
applications/tests/avoidance/pyavoidance.cxx.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/pyavoidance.dir/build.make CMakeFiles/pyavoidance.dir/applications/tests/avoidance/pyavoidance.cxx.o
.PHONY : applications/tests/avoidance/pyavoidance.cxx.o

applications/tests/avoidance/pyavoidance.i: applications/tests/avoidance/pyavoidance.cxx.i
.PHONY : applications/tests/avoidance/pyavoidance.i

# target to preprocess a source file
applications/tests/avoidance/pyavoidance.cxx.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/pyavoidance.dir/build.make CMakeFiles/pyavoidance.dir/applications/tests/avoidance/pyavoidance.cxx.i
.PHONY : applications/tests/avoidance/pyavoidance.cxx.i

applications/tests/avoidance/pyavoidance.s: applications/tests/avoidance/pyavoidance.cxx.s
.PHONY : applications/tests/avoidance/pyavoidance.s

# target to generate assembly for a file
applications/tests/avoidance/pyavoidance.cxx.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/pyavoidance.dir/build.make CMakeFiles/pyavoidance.dir/applications/tests/avoidance/pyavoidance.cxx.s
.PHONY : applications/tests/avoidance/pyavoidance.cxx.s

lib/avoidance/Avoidance.o: lib/avoidance/Avoidance.cpp.o
.PHONY : lib/avoidance/Avoidance.o

# target to build an object file
lib/avoidance/Avoidance.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/avoidance/Avoidance.cpp.o
.PHONY : lib/avoidance/Avoidance.cpp.o

lib/avoidance/Avoidance.i: lib/avoidance/Avoidance.cpp.i
.PHONY : lib/avoidance/Avoidance.i

# target to preprocess a source file
lib/avoidance/Avoidance.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/avoidance/Avoidance.cpp.i
.PHONY : lib/avoidance/Avoidance.cpp.i

lib/avoidance/Avoidance.s: lib/avoidance/Avoidance.cpp.s
.PHONY : lib/avoidance/Avoidance.s

# target to generate assembly for a file
lib/avoidance/Avoidance.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/avoidance/Avoidance.cpp.s
.PHONY : lib/avoidance/Avoidance.cpp.s

lib/cogip_defs/Coords.o: lib/cogip_defs/Coords.cpp.o
.PHONY : lib/cogip_defs/Coords.o

# target to build an object file
lib/cogip_defs/Coords.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/cogip_defs/Coords.cpp.o
.PHONY : lib/cogip_defs/Coords.cpp.o

lib/cogip_defs/Coords.i: lib/cogip_defs/Coords.cpp.i
.PHONY : lib/cogip_defs/Coords.i

# target to preprocess a source file
lib/cogip_defs/Coords.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/cogip_defs/Coords.cpp.i
.PHONY : lib/cogip_defs/Coords.cpp.i

lib/cogip_defs/Coords.s: lib/cogip_defs/Coords.cpp.s
.PHONY : lib/cogip_defs/Coords.s

# target to generate assembly for a file
lib/cogip_defs/Coords.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/cogip_defs/Coords.cpp.s
.PHONY : lib/cogip_defs/Coords.cpp.s

lib/obstacles/Obstacle.o: lib/obstacles/Obstacle.cpp.o
.PHONY : lib/obstacles/Obstacle.o

# target to build an object file
lib/obstacles/Obstacle.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/obstacles/Obstacle.cpp.o
.PHONY : lib/obstacles/Obstacle.cpp.o

lib/obstacles/Obstacle.i: lib/obstacles/Obstacle.cpp.i
.PHONY : lib/obstacles/Obstacle.i

# target to preprocess a source file
lib/obstacles/Obstacle.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/obstacles/Obstacle.cpp.i
.PHONY : lib/obstacles/Obstacle.cpp.i

lib/obstacles/Obstacle.s: lib/obstacles/Obstacle.cpp.s
.PHONY : lib/obstacles/Obstacle.s

# target to generate assembly for a file
lib/obstacles/Obstacle.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/obstacles/Obstacle.cpp.s
.PHONY : lib/obstacles/Obstacle.cpp.s

lib/obstacles/ObstacleCircle.o: lib/obstacles/ObstacleCircle.cpp.o
.PHONY : lib/obstacles/ObstacleCircle.o

# target to build an object file
lib/obstacles/ObstacleCircle.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/obstacles/ObstacleCircle.cpp.o
.PHONY : lib/obstacles/ObstacleCircle.cpp.o

lib/obstacles/ObstacleCircle.i: lib/obstacles/ObstacleCircle.cpp.i
.PHONY : lib/obstacles/ObstacleCircle.i

# target to preprocess a source file
lib/obstacles/ObstacleCircle.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/obstacles/ObstacleCircle.cpp.i
.PHONY : lib/obstacles/ObstacleCircle.cpp.i

lib/obstacles/ObstacleCircle.s: lib/obstacles/ObstacleCircle.cpp.s
.PHONY : lib/obstacles/ObstacleCircle.s

# target to generate assembly for a file
lib/obstacles/ObstacleCircle.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/obstacles/ObstacleCircle.cpp.s
.PHONY : lib/obstacles/ObstacleCircle.cpp.s

lib/obstacles/ObstaclePolygon.o: lib/obstacles/ObstaclePolygon.cpp.o
.PHONY : lib/obstacles/ObstaclePolygon.o

# target to build an object file
lib/obstacles/ObstaclePolygon.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/obstacles/ObstaclePolygon.cpp.o
.PHONY : lib/obstacles/ObstaclePolygon.cpp.o

lib/obstacles/ObstaclePolygon.i: lib/obstacles/ObstaclePolygon.cpp.i
.PHONY : lib/obstacles/ObstaclePolygon.i

# target to preprocess a source file
lib/obstacles/ObstaclePolygon.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/obstacles/ObstaclePolygon.cpp.i
.PHONY : lib/obstacles/ObstaclePolygon.cpp.i

lib/obstacles/ObstaclePolygon.s: lib/obstacles/ObstaclePolygon.cpp.s
.PHONY : lib/obstacles/ObstaclePolygon.s

# target to generate assembly for a file
lib/obstacles/ObstaclePolygon.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/obstacles/ObstaclePolygon.cpp.s
.PHONY : lib/obstacles/ObstaclePolygon.cpp.s

lib/obstacles/ObstacleRectangle.o: lib/obstacles/ObstacleRectangle.cpp.o
.PHONY : lib/obstacles/ObstacleRectangle.o

# target to build an object file
lib/obstacles/ObstacleRectangle.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/obstacles/ObstacleRectangle.cpp.o
.PHONY : lib/obstacles/ObstacleRectangle.cpp.o

lib/obstacles/ObstacleRectangle.i: lib/obstacles/ObstacleRectangle.cpp.i
.PHONY : lib/obstacles/ObstacleRectangle.i

# target to preprocess a source file
lib/obstacles/ObstacleRectangle.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/obstacles/ObstacleRectangle.cpp.i
.PHONY : lib/obstacles/ObstacleRectangle.cpp.i

lib/obstacles/ObstacleRectangle.s: lib/obstacles/ObstacleRectangle.cpp.s
.PHONY : lib/obstacles/ObstacleRectangle.s

# target to generate assembly for a file
lib/obstacles/ObstacleRectangle.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/obstacles/ObstacleRectangle.cpp.s
.PHONY : lib/obstacles/ObstacleRectangle.cpp.s

lib/path/Pose.o: lib/path/Pose.cpp.o
.PHONY : lib/path/Pose.o

# target to build an object file
lib/path/Pose.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/path/Pose.cpp.o
.PHONY : lib/path/Pose.cpp.o

lib/path/Pose.i: lib/path/Pose.cpp.i
.PHONY : lib/path/Pose.i

# target to preprocess a source file
lib/path/Pose.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/path/Pose.cpp.i
.PHONY : lib/path/Pose.cpp.i

lib/path/Pose.s: lib/path/Pose.cpp.s
.PHONY : lib/path/Pose.s

# target to generate assembly for a file
lib/path/Pose.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/path/Pose.cpp.s
.PHONY : lib/path/Pose.cpp.s

lib/utils/utils.o: lib/utils/utils.cpp.o
.PHONY : lib/utils/utils.o

# target to build an object file
lib/utils/utils.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/utils/utils.cpp.o
.PHONY : lib/utils/utils.cpp.o

lib/utils/utils.i: lib/utils/utils.cpp.i
.PHONY : lib/utils/utils.i

# target to preprocess a source file
lib/utils/utils.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/utils/utils.cpp.i
.PHONY : lib/utils/utils.cpp.i

lib/utils/utils.s: lib/utils/utils.cpp.s
.PHONY : lib/utils/utils.s

# target to generate assembly for a file
lib/utils/utils.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/avoidance_lib.dir/build.make CMakeFiles/avoidance_lib.dir/lib/utils/utils.cpp.s
.PHONY : lib/utils/utils.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... avoidance_lib"
	@echo "... pyavoidance"
	@echo "... test_avoidance"
	@echo "... applications/tests/avoidance/main.o"
	@echo "... applications/tests/avoidance/main.i"
	@echo "... applications/tests/avoidance/main.s"
	@echo "... applications/tests/avoidance/pyavoidance.o"
	@echo "... applications/tests/avoidance/pyavoidance.i"
	@echo "... applications/tests/avoidance/pyavoidance.s"
	@echo "... lib/avoidance/Avoidance.o"
	@echo "... lib/avoidance/Avoidance.i"
	@echo "... lib/avoidance/Avoidance.s"
	@echo "... lib/cogip_defs/Coords.o"
	@echo "... lib/cogip_defs/Coords.i"
	@echo "... lib/cogip_defs/Coords.s"
	@echo "... lib/obstacles/Obstacle.o"
	@echo "... lib/obstacles/Obstacle.i"
	@echo "... lib/obstacles/Obstacle.s"
	@echo "... lib/obstacles/ObstacleCircle.o"
	@echo "... lib/obstacles/ObstacleCircle.i"
	@echo "... lib/obstacles/ObstacleCircle.s"
	@echo "... lib/obstacles/ObstaclePolygon.o"
	@echo "... lib/obstacles/ObstaclePolygon.i"
	@echo "... lib/obstacles/ObstaclePolygon.s"
	@echo "... lib/obstacles/ObstacleRectangle.o"
	@echo "... lib/obstacles/ObstacleRectangle.i"
	@echo "... lib/obstacles/ObstacleRectangle.s"
	@echo "... lib/path/Pose.o"
	@echo "... lib/path/Pose.i"
	@echo "... lib/path/Pose.s"
	@echo "... lib/utils/utils.o"
	@echo "... lib/utils/utils.i"
	@echo "... lib/utils/utils.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

