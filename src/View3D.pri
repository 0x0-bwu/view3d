ROOT_DIR = $$dirname(PWD)
BUILD_OUTPUT = $$dirname(PWD)/build/debug
CONFIG(release, debug | release) {
    BUILD_OUTPUT = $$dirname(PWD)/build/release
}

BOOST_PATH = /home/bwu/code/3rdparty/boost_1_82_0
BOOST_INCLUDE_PATH = $$BOOST_PATH/include
BOOST_LIBRARY_PATH = $$BOOST_PATH/lib

EIGEN_LIBRARY_PATH = /usr/include/eigen3

BUILD_LIB_PATH = $$BUILD_OUTPUT/bin
GENERIC_LIB_PATH = $$ROOT_DIR/thirdpart
