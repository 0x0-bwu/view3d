#!usr/bin/bash
qmake src/View3D.pro -spec linux-g++ CONFIG+=debug CONFIG+=qml_debug && /usr/bin/make qmake_all
make