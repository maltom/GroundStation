#include "gsmainwindow.h"
#include "ui_gsmainwindow.h"
#include <opencv2/opencv.hpp>
#include "videoprocess.h"
#include "spacemousecontroller.h"
#include "positiondata.h"
#include <QTimer>
#include <chrono>

GSMainWindow::GSMainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::GSMainWindow)
{

    /*cv::Mat inputImage = cv::imread("Kasia.jpg");
    if(!inputImage.empty()) cv::imshow("Display Image", inputImage);*/

    ui->setupUi(this);

    videoStart();
    spaceMouseStart();

    modeButtonsInitialization();
}

GSMainWindow::~GSMainWindow()
{
    videoThread->quit();
    while(!videoThread->isFinished());

    spaceMouseThread->quit();
    while(!videoThread->isFinished());

    delete videoThread;
    delete ui;
}

void GSMainWindow::videoStart()
{
    videoThread = new QThread();
    videoProcess *videoStream = new videoProcess();
    QTimer *videoTriggerTimer = new QTimer();

    videoTriggerTimer->setInterval(1);

    connect(videoTriggerTimer,SIGNAL(timeout()),videoStream,SLOT(receiveCameraFrame()));
    connect(this,SIGNAL(sendVideoSetup(int)),videoStream,SLOT(receiveCameraSetup(int)));
    connect(videoStream,SIGNAL(sendCameraFrame(QImage)),this,SLOT(receiveCameraFrame(QImage)));
    connect(videoStream,SIGNAL(sendCameraStatus(int)),this,SLOT(receiveCameraStatus(int)));

    connect(videoThread,SIGNAL(finished()),videoStream,SLOT(deleteLater()));
    connect(videoThread,SIGNAL(finished()),videoTriggerTimer,SLOT(deleteLater()));

    videoTriggerTimer->start();
    videoStream->moveToThread(videoThread);
    videoTriggerTimer->moveToThread(videoThread);

    videoThread->start();

    emit sendVideoSetup(cameraChosen);
}
void GSMainWindow::spaceMouseStart()
{
    spaceMouseThread = new QThread();
    spaceMouseController *spaceMouse = new spaceMouseController();

    QTimer *spaceMouseTriggerTimer = new QTimer();

    spaceMouseTriggerTimer->setInterval(1);

    connect(spaceMouseTriggerTimer,SIGNAL(timeout()),spaceMouse,SLOT(receiveCoordinates()));
    connect(spaceMouse,SIGNAL(sendCoordinates(int, int, int, int, int, int)),this, SLOT(receiveCoordinates(int, int, int, int, int, int)));
    connect(spaceMouse,SIGNAL(sendSpaceStatus(int)),this,SLOT(receiveSpaceStatus(int)));

    connect(spaceMouseThread,SIGNAL(finished()),spaceMouse,SLOT(deleteLater()));
    connect(spaceMouseThread,SIGNAL(finished()),spaceMouseTriggerTimer,SLOT(deleteLater()));

    spaceMouseTriggerTimer->start();
    spaceMouse->moveToThread(spaceMouseThread);
    spaceMouseTriggerTimer->moveToThread(spaceMouseThread);
    spaceMouseThread->start();
}

void GSMainWindow::modeButtonsInitialization(void)
{
    connect(ui->toggleCameraButton,SIGNAL(released()),SLOT(changeCamera()));
    connect(ui->toggleSteeringModeButton,SIGNAL(released()),SLOT(changeSteeringMode()));
}

void GSMainWindow::receiveCameraFrame(QImage frame)
{
    ui->label->setPixmap(QPixmap::fromImage(frame));

}


void GSMainWindow::receiveCoordinates(int x, int y, int z, int roll, int pitch, int yaw)
{
    spaceMousePositionData.setPositionAndVelocity("current", x, y, z, roll, pitch, yaw);
    spaceMousePositionData.recalculateSpaceMousePosition();
    printSpaceMouseCoordinates();
    setTargetPosition();
}
void GSMainWindow::setTargetPosition(void)
{
    std::vector<double> vec = spaceMousePositionData.getPositionAndVelocity("current","position");
    spaceMousePositionData.calculateStep(vec, steeringMode);
    rovPosition.addToPositionAndVelocity("future",vec);
    printSetTargetPosition();
}

void GSMainWindow::printSpaceMouseCoordinates(void)
{
    std::vector<double> data = spaceMousePositionData.getPositionAndVelocity("current","position");
    if(!data.empty())
    {
        ui->spaceMouseXValue->setText(QString::number((int)data[0]));
        ui->spaceMouseYValue->setText(QString::number((int)data[1]));
        ui->spaceMouseZValue->setText(QString::number((int)data[2]));
        ui->spaceMouseRollValue->setText(QString::number((int)data[3]));
        ui->spaceMousePitchValue->setText(QString::number((int)data[4]));
        ui->spaceMouseYawValue->setText(QString::number((int)data[5]));
    }
}
void GSMainWindow::printSetTargetPosition(void)
{
    std::vector<double> data = rovPosition.getPositionAndVelocity("future","position");
    if(!data.empty())
    {
        ui->setXValue->setText(QString::number((int)(data[0]*100)));
        ui->setYValue->setText(QString::number((int)(data[1]*100)));
        ui->setZValue->setText(QString::number((int)(data[2]*100)));
        ui->setRollValue->setText(QString::number((int)(data[3]*100)));
        ui->setPitchValue->setText(QString::number((int)(data[4]*100)));
        ui->setYawValue->setText(QString::number((int)(data[5]*100)));
    }
}


void GSMainWindow::receiveSpaceStatus(int status)
{
    if(status)
        ui->spaceStatusLabel->setStyleSheet(("QLabel { background-color : darkGreen;}"));
    else
        ui->spaceStatusLabel->setStyleSheet(("QLabel { background-color : darkRed;}"));

}

void GSMainWindow::receiveCameraStatus(int status)
{

    if(status)
        ui->cameraStatusLabel->setStyleSheet(("QLabel { background-color : darkGreen;}"));
    else
        ui->cameraStatusLabel->setStyleSheet(("QLabel { background-color : darkRed;}"));
}


void GSMainWindow::changeCamera(void)
{
    ++cameraChosen;
    if(cameraChosen >= numberOfCams)
    {
        cameraChosen = 0;
    }
    emit sendVideoSetup(cameraChosen);
}

void GSMainWindow::changeSteeringMode(void)
{
    (steeringMode == 1) ? steeringMode = 0 : steeringMode = 1;
if(steeringMode)
    ui->XTextLabel->setText("PICZ");
else
    ui->XTextLabel->setText("CZIP");
}

