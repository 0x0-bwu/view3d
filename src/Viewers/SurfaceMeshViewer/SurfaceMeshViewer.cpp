#include <QApplication>
#include "ModelView.h"
#include <iostream>
int main(int argc, char *argv[])
{
    QApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
    QApplication a(argc, argv);

    using namespace view;

    SurfaceMeshView viewer;

    viewer.setWindowTitle("surface mesh");
    viewer.show();

    return a.exec();
}
