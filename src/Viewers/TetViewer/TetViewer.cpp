#include <QApplication>
#include "ModelView.h"
#include <iostream>
int main(int argc, char *argv[])
{
    QApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
    QApplication a(argc, argv);

    using namespace view;

    TetrahedronModelView viewer;
    
    viewer.setWindowTitle("tetrahedron");
    viewer.show();

    return a.exec();
}
