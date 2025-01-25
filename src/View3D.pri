ROOT_DIR = $$dirname(PWD)
BUILD_OUTPUT = $$dirname(PWD)/build/debug
CONFIG(release, debug | release) {
    BUILD_OUTPUT = $$dirname(PWD)/build/release
}

BUILD_LIB_PATH = $$BUILD_OUTPUT/bin
GENERIC_LIB_PATH = $$ROOT_DIR/thirdpart
EIGEN_PATH = /home/bing/code/3rdparty/eigen