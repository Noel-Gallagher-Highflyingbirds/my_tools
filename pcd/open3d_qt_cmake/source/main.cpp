#include "registration_qt.h"
#include <QtWidgets/QApplication>
#include <QCoreApplication>
#include <iostream>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    open3d_qt w;
    w.show();
    return a.exec();
}
