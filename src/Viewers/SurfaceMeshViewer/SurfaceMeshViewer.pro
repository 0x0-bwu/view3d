greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
QT += core gui
CONFIG += c++1z
TEMPLATE = app
TARGET   = SurfaceMeshViewer
SOURCES  = SurfaceMeshViewer.cpp
include(../../View3D.pri)

INCLUDEPATH = ./ \
              ../../ViewLib \
              $$GENERIC_LIB_PATH

LIBS += -L$$BUILD_LIB_PATH -lView3D

DESTDIR     = $$BUILD_OUTPUT/bin
OBJECTS_DIR = $$BUILD_OUTPUT/obj/$${TARGET}/
MOC_DIR     = $$BUILD_OUTPUT/moc/$${TARGET}/
RCC_DIR     = $$BUILD_OUTPUT/rcc/$${TARGET}/
UI_DIR      = $$BUILD_OUTPUT/ui/$${TARGET}/
