#include <QApplication>
#include "ModelView.h"
#include <iostream>
int main(int argc, char *argv[])
{
    QApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
    QApplication a(argc, argv);

    using namespace view;

//     FrameModelView viewer;
//     SurfaceModelView viewer;
    TetrahedronModelView viewer;
    // SurfaceMeshView viewer;
    viewer.setWindowTitle("test");
    viewer.show();

    return a.exec();
}
