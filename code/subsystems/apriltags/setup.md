To run files inside src/
ex. test.cpp
make test
./test

setup for https://april.eecs.umich.edu/software/apriltag.html

Installed packages based on niftk apriltags library
sudo apt-get install cmake libopencv-dev libeigen3-dev libv4l-dev

Followed readme for build instructions

BOOST INSTALLATION
attempted 3/16,2pm: apt-get install libboost-python-dev (132mb)

may not need this one?
sudo apt-get install python-dev libxml2-dev libxslt-dev

To build the C++ Apriltag boost library:
make clean && make TagDetector.so



apriltags library setup (for Niftk apriltags library)

clone niftk apriltags library
cd apriltags
make

>> should get no such file/dir for Eigen/Dense
http://stackoverflow.com/questions/23284473/fatal-error-eigen-dense-no-such-file-or-directory
FIX:
cd /usr/include/
sudo ln -sf eigen3/Eigen Eigen
ALTERNATE:
replace #include <Eigen/XXX> with #include <eigen3/Eigen/XXX>