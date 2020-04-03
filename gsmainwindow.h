#ifndef GSMAINWINDOW_H
#define GSMAINWINDOW_H


#include <QMainWindow>
#include <QThread>
#include "positiondata.h"

QT_BEGIN_NAMESPACE
namespace Ui { class GSMainWindow; }
QT_END_NAMESPACE

class GSMainWindow : public QMainWindow
{
    Q_OBJECT

private:
    Ui::GSMainWindow *ui;
    //Threads
    QThread *videoThread;
    QThread *spaceMouseThread;
    //Technical Values
    const int numberOfCams = 2;

    //Modes
    int steeringMode = 0;                       // fast = 0, precise = 1
    int cameraChosen = 0;                       // frontal camera = 0, downward camera = 0
    //positionData
    positionData spaceMousePositionData;        // only current is in use
    positionData rovPosition;                   // past - previous timestep, current - present time step, future - set position
    //Startup functions
    void videoStart(void);
    void spaceMouseStart(void);
    void modeButtonsInitialization(void);
    //Casual functions
    void setTargetPosition(void);

public:
    GSMainWindow(QWidget *parent = nullptr);
    ~GSMainWindow();

private slots:
    void receiveCameraFrame(QImage frame);
    void receiveCoordinates(int x, int y, int z,
                            int roll, int pitch, int yaw);
    void receiveSpaceStatus(int status);
    void receiveCameraStatus(int status);

    void changeCamera(void);
    void changeSteeringMode(void);

    void printSpaceMouseCoordinates(void);
    void printSetTargetPosition(void);

signals:
    void sendVideoSetup(int device);



    //void

};
#endif // GSMAINWINDOW_H
