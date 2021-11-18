#!usr/bin/bash
qmake src/View3D.pro -spec linux-g++ CONFIG+=release CONFIG+=qml_debug && /usr/bin/make qmake_all
make
