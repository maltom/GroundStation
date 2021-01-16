#include "gsmainwindow.h"
#include <ros/ros.h>
#include <QApplication>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc,argv,"GroundStation");
    GSMainWindow w;

    w.show();

    return a.exec();
}
