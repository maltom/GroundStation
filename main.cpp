#include "gsmainwindow.h"
#include <ros/ros.h>
#include <QApplication>

int main( int argc, char* argv[] )
{
    QApplication a( argc, argv );
    GSMainWindow w;

    w.show();

    return a.exec();
}
