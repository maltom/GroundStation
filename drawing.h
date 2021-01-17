#ifndef DRAWING_H
#define DRAWING_H

#include <QImage>
#include <QObject>

class drawing : public QObject
{
    Q_OBJECT
public:
    explicit drawing(QObject *parent = nullptr);

private:
    QImage base = QImage(":/images/resources/images/look.png");
    QImage orientation = base;

signals:
    void sendPictureToDraw(QImage pic);

public slots:
    void receivePositionsToPicture(double x11, double y11, double x21, double y21);

};

#endif // DRAWING_H
