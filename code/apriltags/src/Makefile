CC = gcc
CXX = g++

# Add apriltags core library to path, requires -I
APRILTAGS = ../apriltags/

CPPFLAGS = -I..
CFLAGS = -g -std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -O4
CXXFLAGS = -g -Wall -O4 -I$(APRILTAGS) -std=c++11
LDFLAGS = -lpthread -lm

TARGETS := apriltag_demo opencv_demo test

.PHONY: all
all: apriltag_demo

test: test.o ../apriltags/libapriltag.a TagDetector.o
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS) `pkg-config --libs opencv`

apriltag_demo: apriltag_demo.o ../libapriltag.a
	@echo "   [$@]"
	@$(CC) -o $@ $^ $(LDFLAGS)

opencv_demo: opencv_demo.o ../libapriltag.a
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS) `pkg-config --libs opencv`

%.o: %.c
	@echo "   $@"
	@$(CC) -o $@ -c $< $(CFLAGS) $(CPPFLAGS)

%.o: %.cc
	@echo "   $@"
	@$(CXX) -o $@ -c $< $(CXXFLAGS) $(CPPFLAGS)

%.o: %.cpp
	@echo "   $@"
	@$(CXX) -o $@ -c $< $(CXXFLAGS) $(CPPFLAGS)

.PHONY: clean
clean:
	@rm -rf *.o $(TARGETS)
