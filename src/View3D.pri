ROOT_DIR = ..
BUILD_OUTPUT = $$ROOT_DIR/build/debug
CONFIG(release, debug | release) {
    BUILD_OUTPUT = $$ROOT_DIR/build/release
}

BUILD_LIB_PATH = BUILD_OUTPUT
