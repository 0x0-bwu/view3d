#!usr/bin/bash
/home/bwu/Qt5.9.2/5.9.2/gcc_64/bin/qmake /mnt/c/Users/bwu/iCloudDrive/Code/Garage/src/view3d/View3D.pro -spec linux-g++ CONFIG+=release CONFIG+=qml_debug && /usr/bin/make qmake_all
make
