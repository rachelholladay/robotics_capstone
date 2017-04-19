Installation of libraries using Ubuntu 14.04:

Using Python 2.7.6

Boost and Apriltags: See subsystems/apriltags setup.md

OpenCV for python
pip install opencv-python

Other libraries:
scipy



Protobuf:
    pip install protobuf
Protobuf Compiler:
    apt-get install protobuf-compiler
Raspberry Pi GPIO usage:
    pip install RPi.GPIO


Compiling .proto files:
    protoc $SRC_FILE --python_out=$OUT_DIR
Compilation should occur inside the messages/ folder, so $OUT_DIR=.


