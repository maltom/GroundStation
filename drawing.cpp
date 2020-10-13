#include "drawing.h"
#include <QPainter>
#include <QImage>

drawing::drawing(QObject *parent) : QObject(parent)
{

}
void drawing::receivePositionsToPicture(double x11, double y11, double x21, double y21)
{
    orientation = base;
    QPainter *draw = new QPainter(&orientation);
    QPen pen;
    draw->setRenderHint(QPainter::Antialiasing, true);

    //draw->drawLine(320,111,1000,200);
    int x10 = 320, y10 = 111, x20 = 960, y20 = 111;
    int intx11 = (320-static_cast<int>(3.0/8.0*x11*1000.0));
    int inty11 = (111+static_cast<int>(3.0/8.0*y11*1000.0));
    if(intx11<0)
        intx11=0;
    else if (x11>1280)
        intx11 = 1280;

    if(inty11<0)
        inty11=0;
    else if (inty11>211)
        inty11 = 211;

    int intx21 = (960-static_cast<int>(3.0/8.0*x21*1000.0));
    int inty21 = (111-static_cast<int>(3.0/8.0*y21*1000.0));
    if(intx21<0)
        intx21=0;
    else if (x21>1280)
        intx21 = 1280;

    if(inty21<0)
        inty21=0;
    else if (inty21>211)
        inty21 = 211;

    pen.setColor(QColor::fromRgb(0,105,60));
    pen.setWidth(3);
    pen.setStyle(Qt::PenStyle::DashLine);
    pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    draw->setPen(pen);

    draw->drawLine(x10,y10,intx11,inty11);
    draw->drawLine(x20,y20,intx21,inty21);

    pen.setColor(QColor::fromRgb(167,25,48));
    pen.setWidth(8);
    draw->setPen(pen);
    draw->drawPoint(intx11,inty11);
    draw->drawPoint(intx21,inty21);
    emit sendPictureToDraw(orientation);
    delete draw;
}
