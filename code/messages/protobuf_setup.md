Installing protobuf and probotuf compiler:
    pip install protobuf
    apt-get install protobuf-compiler

To compile .proto files:
protoc $SRC_FILE --python_out=$OUT_DIR

For compiling protobuf messages here, $OUT_DIR=.
