#ifndef SPACEMOUSECONTROLLER_H
#define SPACEMOUSECONTROLLER_H

#include <QObject>
#include <libevdev/libevdev.h>

class spaceMouseController : public QObject
{
    Q_OBJECT
public:
    explicit spaceMouseController(QObject *parent = nullptr);

private:
    struct libevdev *dev = NULL;
    int fd;
    char eventName[20] = {};
    std::string getName = "";
    int status = 0;
    int x = 0, y = 0, z = 0;
    int pitch = 0, roll = 0, yaw = 0;
    struct input_event ev;
    void findDevice(void);


public:

    ~spaceMouseController();

signals:
    void sendCoordinates(int x, int y, int z,
                         int roll, int pitch, int yaw);
    void sendSpaceStatus(int status);
    void sendCameraChange(void);
private slots:
    void receiveCoordinates(void);




};

#endif // SPACEMOUSECONTROLLER_H
