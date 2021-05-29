#include "gsmainwindow.h"
#include <ros/ros.h>
#include <QApplication>
#include <QTextCodec>

int main( int argc, char* argv[] )
{
    QApplication a( argc, argv );
    GSMainWindow w;

    QTextCodec::setCodecForLocale( QTextCodec::codecForName( "UTF-8" ) );

    w.show();

    return a.exec();
}
