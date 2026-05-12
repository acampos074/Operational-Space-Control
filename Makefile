# This Makefile assumes that you have GLFW libraries and headers installed on,
# which is commonly available through your distro's package manager.
# On Debian and Ubuntu, GLFW can be installed via `apt install libglfw3-dev`.
# ./robot.cpp -O2

COMMON=-O2 -I../include ./robot.cpp -L../lib -std=c++17 -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/../lib
#$(CXX) $(COMMON) basic.cc              -lmujoco -lglfw   -o basic
.PHONY: all
all:
	g++ $(COMMON) controller.cpp -lmujoco -lglfw -o controller
