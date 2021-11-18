#-------------------------------------------------
#
# Project created by QtCreator 2021-05-26T06:24:32
#
#-------------------------------------------------

QT += core gui
QT += opengl

TARGET = View3D
TEMPLATE = app
CONFIG += c++1z
QMAKE_CXXFLAGS -= -Wall
QMAKE_CXXFLAGS += -w

BUILD_OUTPUT = "build/debug"
CONFIG(release, debug | release) {
    BUILD_OUTPUT = "build/release"
}

INCLUDEPATH = ./ \
              ../ \
              ../generic \

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
    ShaderView.h \
    View3D.h \
    renderer/IndexBuffer.h \
    renderer/Renderer.h \
    renderer/Shader.h \
    renderer/Texture.h \
    renderer/VertexArray.h \
    renderer/VertexBuffer.h \
    renderer/VertexBufferLayout.h

SOURCES += \
    ../mesher/Mesher2D.cpp \
    ../mesher/MeshFileUtility.cpp \
    ShaderView.cpp \
     \
    renderer/IndexBuffer.cpp \
    renderer/Renderer.cpp \
    renderer/Shader.cpp \
    renderer/Texture.cpp \
    renderer/VertexArray.cpp \
    renderer/VertexBuffer.cpp
SOURCES += \
    Action.cpp \
    Camera.cpp \
    Frame.cpp \
    Model.cpp \
    ModelView.cpp \
    Painter.cpp \
    View3D.cpp \
    main.cpp
 
DESTDIR     = ../../$${BUILD_OUTPUT}/bin
OBJECTS_DIR = ../../$${BUILD_OUTPUT}/obj/$${TARGET}/
MOC_DIR     = ../../$${BUILD_OUTPUT}/moc/$${TARGET}/
RCC_DIR     = ../../$${BUILD_OUTPUT}/rcc/$${TARGET}/
UI_DIR      = ../../$${BUILD_OUTPUT}/ui/$${TARGET}/

RESOURCES += \
    res.qrc
