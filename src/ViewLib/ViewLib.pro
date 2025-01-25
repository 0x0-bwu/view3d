QT += core gui
QT += opengl

TARGET = View3D
TEMPLATE = lib
CONFIG += c++1z
QMAKE_CXXFLAGS -= -Wall
QMAKE_CXXFLAGS += -w

include(../View3D.pri)

INCLUDEPATH = ./ \
              ../ \
              $$GENERIC_LIB_PATH \
              $$EIGEN_PATH

win32:{
INCLUDEPATH += C:\\Dev\\include\\
LIBS += -L"C:\\Dev\\libs" \
        -lopengl32 \
        -lglfw3 \
        -lglu32 \

LIBS += -lOpengl32
}
unix:{
LIBS += -L"/usr/lib/x86_64-linux-gnu/" \
        -lGL\
        -lGLU \
        -lGLEW \

BOOST_LIBRARY = "/usr/local/lib/"
LIBS += -L$$BOOST_LIBRARY \
        -lboost_serialization \
}
    
HEADERS += \
    Action.h \
    Camera.h \
    Color.h \
    Config.h \
    Frame.h \
    Model.h \
    ModelView.h \
    Painter.h \
    View3D.h

SOURCES += \
    Action.cpp \
    Camera.cpp \
    Frame.cpp \
    Model.cpp \
    ModelView.cpp \
    Painter.cpp \
    View3D.cpp

DESTDIR     = $$BUILD_OUTPUT/bin
OBJECTS_DIR = $$BUILD_OUTPUT/obj/$${TARGET}/
MOC_DIR     = $$BUILD_OUTPUT/moc/$${TARGET}/
RCC_DIR     = $$BUILD_OUTPUT/rcc/$${TARGET}/
UI_DIR      = $$BUILD_OUTPUT/ui/$${TARGET}/

