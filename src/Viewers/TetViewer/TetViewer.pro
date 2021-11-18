TEMPLATE = app
TARGET   = TetViewer
SOURCES  = TetViewer.cpp
include(../../View3D.pri)

LIBS += -L$$BUILD_LIB_PATH
        -lView3D \
