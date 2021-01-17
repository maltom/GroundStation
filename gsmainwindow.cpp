#include "gsmainwindow.h"
#include "ui_gsmainwindow.h"
#include <opencv2/opencv.hpp>
#include "videoprocess.h"
#include "spacemousecontroller.h"
#include "lqrhandler.h"
#include "rosnodehandler.h"
#include "positiondata.h"
#include "drawing.h"
#include <QTimer>
#include <chrono>
#include <QMetaType>
#include <QMouseEvent>
#include <QPainter>
#include <QImage>

GSMainWindow::GSMainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::GSMainWindow)
{

    qRegisterMetaType<Matrix1212>("Matrix1212");
    qRegisterMetaType<Matrix126>("Matrix612");
    qRegisterMetaType<Matrix612>("Matrix126");
    /*cv::Mat inputImage = cv::imread("Kasia.jpg");
    if(!inputImage.empty()) cv::imshow("Display Image", inputImage);*/

    ui->setupUi(this);

    videoStart();
    spaceMouseStart();

    modeButtonsInitialization();

    drawFirstGraphics();
    drawingStart();

    regulatorStart();
    rosStart();

    //centralWidget()->setAttribute(Qt::WA_TransparentForMouseEvents);
    setMouseTracking(true);


}

GSMainWindow::~GSMainWindow()
{
    videoThread->quit();
    while(!videoThread->isFinished());

    spaceMouseThread->quit();
    while(!videoThread->isFinished());

    drawingThread->quit();
    while(!drawingThread->isFinished());

    regulatorThread->quit();
    while(!regulatorThread->isFinished());

    rosThread->quit();
    while(!rosThread->isFinished());

    delete videoThread;
    delete spaceMouseThread;
    delete drawingThread;
    delete regulatorThread;
    delete rosThread;
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
    connect(spaceMouse,SIGNAL(sendCameraChange()),this,SLOT(changeCamera()));
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


    connect(ui->motorTestEnabled,SIGNAL(stateChanged(int)),this,SLOT(testModeEnable(int)));
    connect(ui->motor1Slider,SIGNAL(valueChanged(int)),SLOT(updateMotorPWMValues()));
    connect(ui->motor2Slider,SIGNAL(valueChanged(int)),SLOT(updateMotorPWMValues()));
    connect(ui->motor3Slider,SIGNAL(valueChanged(int)),SLOT(updateMotorPWMValues()));
    connect(ui->motor4Slider,SIGNAL(valueChanged(int)),SLOT(updateMotorPWMValues()));
    connect(ui->motor5Slider,SIGNAL(valueChanged(int)),SLOT(updateMotorPWMValues()));
    connect(ui->servo1Slider,SIGNAL(valueChanged(int)),SLOT(updateMotorPWMValues()));
    connect(ui->servo2Slider,SIGNAL(valueChanged(int)),SLOT(updateMotorPWMValues()));
}
void GSMainWindow::drawingStart()
{
    drawingThread = new QThread();
    drawing *orientationDrawing = new drawing();

    //connect(this,SIGNAL(sendDrawingPositions(double,double,double,double)),orientationDrawing,SLOT(receivePositionsToPicture(double,double,double,double)));
    connect(this,&GSMainWindow::sendDrawingPositions,orientationDrawing,&drawing::receivePositionsToPicture);
    connect(orientationDrawing,SIGNAL(sendPictureToDraw(QImage)),this,SLOT(receiveOrientationDrawing(QImage)));


    orientationDrawing->moveToThread(drawingThread);

    drawingThread->start();
}
void GSMainWindow::regulatorStart()
{
    regulatorThread = new QThread();
    regulator = new LQRHandler(this->regulatorTickTime,&rovPosition,nullptr);
    QTimer *regulatorTimer = new QTimer();

    regulatorTimer->setInterval(this->regulatorTickTime);

    connect(regulatorTimer,SIGNAL(timeout()),regulator,SLOT(update()));
    connect(regulator,&LQRHandler::positionReady,this,&GSMainWindow::printCurrentPosition);
    regulatorTimer->start();
    regulator->moveToThread(regulatorThread);
    regulatorTimer->moveToThread(regulatorThread);

    regulatorThread->start();
}
void GSMainWindow::rosStart()
{
    rosThread = new QThread();
    rosNodeHandler *rosObj = new rosNodeHandler();

    QTimer *rosTimer = new QTimer();
    rosTimer->setInterval(this->regulatorTickTime);

    connect(regulator,&LQRHandler::positionReady,rosObj,&rosNodeHandler::publishRovParams);
    connect(this,&GSMainWindow::sendTrackBallPosition,rosObj,&rosNodeHandler::publishBallPosition);

    //connect(rosObj,SIGNAL(sendK(Matrix612)),regulator,SLOT(receiveK(Matrix612)));
#ifdef MATLAB
    {
        qRegisterMetaType<Matrix1212>("Matrix1212");
        qRegisterMetaType<Matrix126>("Matrix612");
        qRegisterMetaType<Matrix612>("Matrix126");
        geometry_msgs::Point{Pose.x(),Pose.y(),Pose.z()}
        connect(rosObj,&rosNodeHandler::sendK,regulator,&LQRHandler::receiveK);
        //connect(regulator,SIGNAL(sendAB(Matrix1212, Matrix126)),rosObj,SLOT(publishABToMatlab(Matrix1212, Matrix126)));
        connect(regulator,&LQRHandler::sendAB,rosObj,&rosNodeHandler::publishABToMatlab);
#else
#endif
    connect(rosTimer,&QTimer::timeout,rosObj,&rosNodeHandler::update);

    rosTimer->start();
    rosObj->moveToThread(rosThread);
    rosTimer->moveToThread(regulatorThread);
    //rosObj->update();
    rosThread->start();
    //QTimer::singleShot(1,rosObj,&rosNodeHandler::update);

}
void GSMainWindow::receiveCameraFrame(QImage frame)
{
    ui->label->setPixmap(QPixmap::fromImage(frame));

}
void GSMainWindow::receiveOrientationDrawing(QImage drawing)
{
    ui->orientationLabel->setPixmap(QPixmap::fromImage(drawing));
}

void GSMainWindow::receiveCoordinates(int x, int y, int z, int roll, int pitch, int yaw)
{
    spaceMousePositionData.setPositionAndVelocity("current", x, y, z, roll, pitch, yaw);    // getting coordinates from space mouse and saving it in object
    spaceMousePositionData.recalculateSpaceMousePosition();                                 // normalizing <-350,350> to <-100,100>
    printSpaceMouseCoordinates();                                                           // printing
    setTargetPosition();
}
void GSMainWindow::setTargetPosition(void)
{
    std::vector<double> vec = spaceMousePositionData.getPositionAndVelocity("current","position");  // getting coordinate
    spaceMousePositionData.calculateStep(vec, steeringMode);
    rovPosition.addToPositionAndVelocity("future",vec);
    printSetTargetPosition();
    calculateDeviation();
}
void GSMainWindow::calculateDeviation(void)
{
    deviationPositionData.getDifference("current",rovPosition,"future",rovPosition,"current");
    printDeviation();
}

void GSMainWindow::printSpaceMouseCoordinates(void)
{
    std::vector<double> data = spaceMousePositionData.getPositionAndVelocity("current","position");
    if(!data.empty())
    {
        ui->spaceMouseXValue->setText(QString::number(static_cast<int>(data[0])));
        ui->spaceMouseYValue->setText(QString::number(static_cast<int>(data[1])));
        ui->spaceMouseZValue->setText(QString::number(static_cast<int>(data[2])));
        ui->spaceMouseRollValue->setText(QString::number(static_cast<int>(data[3])));
        ui->spaceMousePitchValue->setText(QString::number(static_cast<int>(data[4])));
        ui->spaceMouseYawValue->setText(QString::number(static_cast<int>(data[5])));
    }
}
void GSMainWindow::printSetTargetPosition(void)
{
    std::vector<double> data = rovPosition.getPositionAndVelocity("future","position");
    if(!data.empty())
    {
        ui->setXValue->setText(QString::number(static_cast<int>(data[0]*100)));
        ui->setYValue->setText(QString::number(static_cast<int>(data[1]*100)));
        ui->setZValue->setText(QString::number(static_cast<int>(data[2]*100)));
        ui->setRollValue->setText(QString::number(static_cast<int>(data[3]*100)));
        ui->setPitchValue->setText(QString::number(static_cast<int>(data[4]*100)));
        ui->setYawValue->setText(QString::number(static_cast<int>(data[5]*100)));
        emit sendDrawingPositions(data[0],data[2],data[0],data[1]);
        emit sendTrackBallPosition(Eigen::Vector3d {data[0],data[1],data[2]});
    }
}
void GSMainWindow::printDeviation(void)
{
    std::vector<double> data = deviationPositionData.getPositionAndVelocity("current","position");
    if(!data.empty())
    {
        ui->differenceXValue->setText(QString::number(static_cast<int>(data[0]*100)));
        ui->differenceYValue->setText(QString::number(static_cast<int>(data[1]*100)));
        ui->differenceZValue->setText(QString::number(static_cast<int>(data[2]*100)));
        ui->differenceRollValue->setText(QString::number(static_cast<int>(data[3]*180/M_PI)));
        ui->differencePitchValue->setText(QString::number(static_cast<int>(data[4]*180/M_PI)));
        ui->differenceYawValue->setText(QString::number(static_cast<int>(data[5]*180/M_PI)));
        //emit sendDrawingPositions(data[0],data[2],data[0],data[1]);
    }
}

void GSMainWindow::printCurrentPosition()
{
    std::vector<double> data = rovPosition.getPositionAndVelocity("current","position");

    if(!data.empty())
    {
        ui->currentXValue->setText(QString::number(static_cast<int>(data[0]*100)));
        ui->currentYValue->setText(QString::number(static_cast<int>(data[1]*100)));
        ui->currentZValue->setText(QString::number(static_cast<int>(data[2]*100)));
        ui->currentRollValue->setText(QString::number(static_cast<int>(data[3]*180/M_PI)));
        ui->currentPitchValue->setText(QString::number(static_cast<int>(data[4]*180/M_PI)));
        ui->currentYawValue->setText(QString::number(static_cast<int>(data[5]*180/M_PI)));
    }
    printDeviation();
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
    if(steeringMode)
    {
        steeringMode = 0;
        ui->precisionLabel->setText("Fast");
    }
    else
    {
        steeringMode = 1;
        ui->precisionLabel->setText("Precise");
    }
}
void GSMainWindow::testModeEnable(int enabled)
{
    ui->motor1Slider->setEnabled(enabled);
    ui->motor2Slider->setEnabled(enabled);
    ui->motor3Slider->setEnabled(enabled);
    ui->motor4Slider->setEnabled(enabled);
    ui->motor5Slider->setEnabled(enabled);
    ui->servo1Slider->setEnabled(enabled);
    ui->servo2Slider->setEnabled(enabled);
    testMode=enabled;
}
void GSMainWindow::updateMotorPWMValues(void)
{
    ui->motor1Value->setText(QString::number(ui->motor1Slider->value()));
    ui->motor2Value->setText(QString::number(ui->motor2Slider->value()));
    ui->motor3Value->setText(QString::number(ui->motor3Slider->value()));
    ui->motor4Value->setText(QString::number(ui->motor4Slider->value()));
    ui->motor5Value->setText(QString::number(ui->motor5Slider->value()));
    ui->servo1Value->setText(QString::number(ui->servo1Slider->value()));
    ui->servo2Value->setText(QString::number(ui->servo2Slider->value()));
}
void GSMainWindow::mouseMoveEvent(QMouseEvent *event)
{

    //ui->XTextLabel->setText(QString(event->source()));
    QCursor::setPos(0,70);
}

void GSMainWindow::drawFirstGraphics(void)
{
    ui->orientationLabel->setPixmap(QPixmap::fromImage(QImage(":/images/resources/images/look.png")));
}

