CXX = g++

# Add apriltags core library to path, requires -I
APRILTAGS = ../apriltags/
APRILTAG_LIB = ../apriltags/libapriltag.a

CPPFLAGS = -I..
CFLAGS = -g -std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -O4
CXXFLAGS = -g -Wall -O4 -I$(APRILTAGS) -std=c++11
LDFLAGS = -lpthread -lm

OPENCV_LIB = `pkg-config --libs opencv`

# Python and Boost dependencies
PYTHON_VERSION = 2.7
PYTHON_INCLUDE = /usr/include/python$(PYTHON_VERSION)
PYTHONFLAGS = -L/usr/lib/python$(PYTHON_VERSION)/config -lpython$(PYTHON_VERSION)
# Location of Boost Python include files/library
BOOST_INC = /usr/include
BOOST_LIB = /usr/lib
BOOSTFLAGS = -shared -Wl,--export-dynamic -L$(BOOST_LIB) -lboost_python

TARGETS := test boost_apriltags

.PHONY: all
all: test boost_apriltags

# TagDetector.so: TagDetector.o $(APRILTAG_LIB)
# 	@echo "   [$@]"
# 	@$(CXX) -o $@ $^ $(LDFLAGS) $(OPENCV_LIB) $(BOOSTFLAGS) $(PYTHONFLAGS)

test: test.o $(APRILTAG_LIB) TagDetector.o
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(OPENCV_LIB) $(BOOSTFLAGS) $(PYTHONFLAGS)

boost_apriltags.so: boost_apriltags.o TagDetector.o $(APRILTAG_LIB)
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(OPENCV_LIB) $(BOOSTFLAGS) $(PYTHONFLAGS) 

# %.o: %.c
# 	@echo "   $@"
# 	@$(CC) -o $@ -c $< $(CFLAGS) $(CPPFLAGS)

# %.o: %.cc
# 	@echo "   $@"
# 	@$(CXX) -o $@ -c $< $(CXXFLAGS) $(CPPFLAGS)

%.o: %.cpp
	@echo "   $@"
	@$(CXX) -o $@ -c $< $(CXXFLAGS) $(CPPFLAGS) -I$(PYTHON_INCLUDE) -I$(BOOST_INC) -fPIC

.PHONY: clean
clean:
	@rm -rf *.o $(TARGETS) *.so
