﻿
#include "gsmainwindow.h"
#include "ui_gsmainwindow.h"
#include <opencv2/opencv.hpp>
#include "videoprocess.h"
#include "spacemousecontroller.h"
#include "lqrhandler.h"
#include "positiondata.h"
#include "drawing.h"
#include "tcpconnectionhandler.h"

#include <QTimer>
#include <chrono>
#include <QMetaType>
#include <QMouseEvent>
#include <QPainter>
#include <QImage>

GSMainWindow::GSMainWindow( QWidget* parent ) : QMainWindow( parent ), ui( new Ui::GSMainWindow )
{

    qRegisterMetaType< Eigen::VectorXd >( "VectorXd" );
    qRegisterMetaType< Eigen::Vector3d >( "Vector3d" );
    qRegisterMetaType< Matrix612 >( "Matrix126" );

    ui->setupUi( this );

    videoStart();
    spaceMouseStart();

    modeButtonsInitialization();

    drawFirstGraphics();
    drawingStart();

    regulatorStart();

    // centralWidget()->setAttribute(Qt::WA_TransparentForMouseEvents);
    setMouseTracking( true );

    connect( this, &GSMainWindow::goPrintSpaceMouseCoordinates, this, &GSMainWindow::printSpaceMouseCoordinates );
    connect( this, &GSMainWindow::goSetTargetPosition, this, &GSMainWindow::setTargetPosition );
    connect( this, &GSMainWindow::goPrintSetTargetPosition, this, &GSMainWindow::printSetTargetPosition );
    connect( this, &GSMainWindow::goCalculateDeviation, this, &GSMainWindow::calculateDeviation );
    connect( this, &GSMainWindow::goPrintDeviation, this, &GSMainWindow::printDeviation );
}

GSMainWindow::~GSMainWindow()
{
    videoThread->quit();
    while( !videoThread->isFinished() )
        ;

    spaceMouseThread->quit();
    while( !spaceMouseThread->isFinished() )
        ;

    drawingThread->quit();
    while( !drawingThread->isFinished() )
        ;

    regulatorThread->quit();
    while( !regulatorThread->isFinished() )
        ;

    tcpComunicationThread->quit();
    while( !tcpComunicationThread->isFinished() )
        ;

    delete videoThread;
    delete spaceMouseThread;
    delete drawingThread;
    delete regulatorThread;
    delete tcpComunicationThread;
    delete ui;
}

void GSMainWindow::videoStart()
{
    videoThread = new QThread();

    videoProcess* videoStream = new videoProcess();
    QTimer* videoTriggerTimer = new QTimer();

    videoTriggerTimer->setInterval( 1 );

    connect( videoTriggerTimer, SIGNAL( timeout() ), videoStream, SLOT( receiveCameraFrame() ) );
    connect( this, SIGNAL( sendVideoSetup( int ) ), videoStream, SLOT( receiveCameraSetup( int ) ) );
    connect( videoStream, SIGNAL( sendCameraFrame( QImage ) ), this, SLOT( receiveCameraFrame( QImage ) ) );
    connect( videoStream, SIGNAL( sendCameraStatus( int ) ), this, SLOT( receiveCameraStatus( int ) ) );

    connect( videoThread, SIGNAL( finished() ), videoStream, SLOT( deleteLater() ) );
    connect( videoThread, SIGNAL( finished() ), videoTriggerTimer, SLOT( deleteLater() ) );

    videoTriggerTimer->start();
    videoStream->moveToThread( videoThread );
    videoTriggerTimer->moveToThread( videoThread );

    videoThread->start();

	emit sendVideoSetup( cameraChosen );
}
void GSMainWindow::spaceMouseStart()
{
    spaceMouseThread                 = new QThread();
    spaceMouseController* spaceMouse = new spaceMouseController();

    QTimer* spaceMouseTriggerTimer = new QTimer();

    spaceMouseTriggerTimer->setInterval( 1 );

    connect( spaceMouseTriggerTimer, SIGNAL( timeout() ), spaceMouse, SLOT( receiveCoordinates() ) );
    connect( spaceMouse,
             SIGNAL( sendCoordinates( int, int, int, int, int, int ) ),
             this,
             SLOT( receiveCoordinates( int, int, int, int, int, int ) ) );
    connect( spaceMouse, SIGNAL( sendSpaceStatus( int ) ), this, SLOT( receiveSpaceStatus( int ) ) );
    connect( spaceMouse, SIGNAL( sendCameraChange() ), this, SLOT( changeCamera() ) );
    connect( spaceMouse, &spaceMouseController::sendCoralProcessingChange, this, &GSMainWindow::toggleCoralProcessing );
    connect( spaceMouseThread, SIGNAL( finished() ), spaceMouse, SLOT( deleteLater() ) );
    connect( spaceMouseThread, SIGNAL( finished() ), spaceMouseTriggerTimer, SLOT( deleteLater() ) );

    spaceMouseTriggerTimer->start();
    spaceMouse->moveToThread( spaceMouseThread );
    spaceMouseTriggerTimer->moveToThread( spaceMouseThread );
    spaceMouseThread->start();
}

void GSMainWindow::tcpHandlerStart( void )
{
    tcpComunicationThread = new QThread();
    connectionHandler     = new tcpConnectionHandler();

    connectionHandler->moveToThread( tcpComunicationThread );

    connect( this, &GSMainWindow::openConnection, connectionHandler, &tcpConnectionHandler::openConnection );
    connect( this, &GSMainWindow::closeConnection, connectionHandler, &tcpConnectionHandler::closeConnection );

    connect( this, &GSMainWindow::openGripper, connectionHandler, &tcpConnectionHandler::sendOpenGripper );
    connect( this, &GSMainWindow::closeGripper, connectionHandler, &tcpConnectionHandler::sendCloseGripper );

    connect( this, &GSMainWindow::openGulper, connectionHandler, &tcpConnectionHandler::sendOpenGulper );
    connect( this, &GSMainWindow::closeGulper, connectionHandler, &tcpConnectionHandler::sendCloseGulper );
    connect( this, &GSMainWindow::forwardGulper, connectionHandler, &tcpConnectionHandler::sendForwardGulper );
    connect( this, &GSMainWindow::backwardGulper, connectionHandler, &tcpConnectionHandler::sendBackwardGulper );
    connect( this, &GSMainWindow::stopGulper, connectionHandler, &tcpConnectionHandler::sendStopGulper );

    connect( this, &GSMainWindow::setPressureRate, connectionHandler, &tcpConnectionHandler::sendPressureRate );
    connect( this, &GSMainWindow::setEulerRate, connectionHandler, &tcpConnectionHandler::sendEulerRate );
    connect( this, &GSMainWindow::setImuRate, connectionHandler, &tcpConnectionHandler::sendImuRate );
}

void GSMainWindow::modeButtonsInitialization( void )
{
    connect( ui->toggleCameraButton, SIGNAL( released() ), SLOT( changeCamera() ) );
    connect( ui->toggleSteeringModeButton, SIGNAL( released() ), SLOT( changeSteeringMode() ) );

    connect( ui->motorTestEnabled, SIGNAL( stateChanged( int ) ), this, SLOT( testModeEnable( int ) ) );
    connect( ui->motor1Slider, SIGNAL( valueChanged( int ) ), SLOT( updateMotorPWMValues() ) );
    connect( ui->motor2Slider, SIGNAL( valueChanged( int ) ), SLOT( updateMotorPWMValues() ) );
    connect( ui->motor3Slider, SIGNAL( valueChanged( int ) ), SLOT( updateMotorPWMValues() ) );
    connect( ui->motor4Slider, SIGNAL( valueChanged( int ) ), SLOT( updateMotorPWMValues() ) );
    connect( ui->motor5Slider, SIGNAL( valueChanged( int ) ), SLOT( updateMotorPWMValues() ) );
    connect( ui->servo1Slider, SIGNAL( valueChanged( int ) ), SLOT( updateMotorPWMValues() ) );
    connect( ui->servo2Slider, SIGNAL( valueChanged( int ) ), SLOT( updateMotorPWMValues() ) );

    connect( ui->connectButton, &QPushButton::released, this, &GSMainWindow::toggleConnection );

    connect( ui->openButton, &QPushButton::released, this, &GSMainWindow::parseOpenGripper );
    connect( ui->closeButton, &QPushButton::released, this, &GSMainWindow::parseCloseGripper );
    connect( ui->clenchButton, &QPushButton::released, this, &GSMainWindow::parseClenchGripper );
    connect( ui->stretchButton, &QPushButton::released, this, &GSMainWindow::parseStretchGripper );

    connect( ui->openGulperButton, &QPushButton::released, this, &GSMainWindow::parseOpenGulper );
    connect( ui->closeGulperButton, &QPushButton::released, this, &GSMainWindow::parseCloseGulper );
    connect( ui->forwardButton, &QPushButton::released, this, &GSMainWindow::parseForwardGulper );
    connect( ui->backwardButton, &QPushButton::released, this, &GSMainWindow::parseBackwardGulper );
    connect( ui->stopButton, &QPushButton::released, this, &GSMainWindow::parseStopGulper );
}
void GSMainWindow::drawingStart()
{
    drawingThread               = new QThread();
    drawing* orientationDrawing = new drawing();

    // connect(this,SIGNAL(sendDrawingPositions(double,double,double,double)),orientationDrawing,SLOT(receivePositionsToPicture(double,double,double,double)));
    connect( this, &GSMainWindow::sendDrawingPositions, orientationDrawing, &drawing::receivePositionsToPicture );
    connect(
        orientationDrawing, SIGNAL( sendPictureToDraw( QImage ) ), this, SLOT( receiveOrientationDrawing( QImage ) ) );

    orientationDrawing->moveToThread( drawingThread );

    drawingThread->start();
}
void GSMainWindow::regulatorStart()
{
    regulatorThread        = new QThread();
    regulator              = new LQRHandler( this->regulatorTickTime, &rovPosition, nullptr );
    QTimer* regulatorTimer = new QTimer();

    regulatorTimer->setInterval( this->regulatorTickTime );

    connect( regulatorTimer, &QTimer::timeout, regulator, &LQRHandler::update );
    connect( regulator, &LQRHandler::positionReady, this, &GSMainWindow::printCurrentPosition );
    regulatorTimer->start();
    regulator->moveToThread( regulatorThread );
    regulatorTimer->moveToThread( regulatorThread );

    regulatorThread->start();
}

void GSMainWindow::receiveCameraFrame( QImage frame )
{
    // 1.7778 - 16:9 screen
    if( static_cast< double >( frame.width() ) / static_cast< double >( frame.height() ) >= 1.7777 )
        ui->label->setPixmap( QPixmap::fromImage( frame.scaledToWidth( 1280 ) ) );
    else
        ui->label->setPixmap( QPixmap::fromImage( frame.scaledToHeight( 960 ) ) );
}
void GSMainWindow::receiveOrientationDrawing( QImage drawing )
{
    ui->orientationLabel->setPixmap( QPixmap::fromImage( drawing ) );
}

void GSMainWindow::receiveCoordinates( int x, int y, int z, int roll, int pitch, int yaw )
{
    spaceMousePositionData.setPositionAndVelocity(
        "current", x, y, z, roll, pitch, yaw ); // getting coordinates from space mouse and saving it in object
    spaceMousePositionData.recalculateSpaceMousePosition(); // normalizing <-350,350> to <-100,100>
    emit goPrintSpaceMouseCoordinates();
    emit goSetTargetPosition();
}
void GSMainWindow::setTargetPosition( void )
{
    std::vector< double > vec
        = spaceMousePositionData.getPositionAndVelocity( "current", "position" ); // getting coordinate
    spaceMousePositionData.calculateStep( vec, steeringMode );
    rovPosition.addToPositionAndVelocity( "future", vec );
    emit goCalculateDeviation();
    emit goPrintSetTargetPosition();
}
void GSMainWindow::calculateDeviation( void )
{
    deviationPositionData.getDifference( "current", rovPosition, "future", rovPosition, "current" );
    emit goPrintDeviation();
}

void GSMainWindow::printSpaceMouseCoordinates( void )
{
    std::vector< double > data = spaceMousePositionData.getPositionAndVelocity( "current", "position" );
    if( !data.empty() )
    {
        ui->spaceMouseXValue->setText( QString::number( static_cast< int >( data[ 0 ] ) ) );
        ui->spaceMouseYValue->setText( QString::number( static_cast< int >( data[ 1 ] ) ) );
        ui->spaceMouseZValue->setText( QString::number( static_cast< int >( data[ 2 ] ) ) );
        ui->spaceMouseRollValue->setText( QString::number( static_cast< int >( data[ 3 ] ) ) );
        ui->spaceMousePitchValue->setText( QString::number( static_cast< int >( data[ 4 ] ) ) );
        ui->spaceMouseYawValue->setText( QString::number( static_cast< int >( data[ 5 ] ) ) );
    }
}
void GSMainWindow::printSetTargetPosition( void )
{
    std::vector< double > data = rovPosition.getPositionAndVelocity( "future", "position" );
    if( !data.empty() )
    {
        ui->setXValue->setText( QString::number( static_cast< int >( data[ 0 ] * 100 ) ) );
        ui->setYValue->setText( QString::number( static_cast< int >( data[ 1 ] * 100 ) ) );
        ui->setZValue->setText( QString::number( static_cast< int >( data[ 2 ] * 100 ) ) );
        ui->setRollValue->setText( QString::number( static_cast< int >( data[ 3 ] * 100 ) ) );
        ui->setPitchValue->setText( QString::number( static_cast< int >( data[ 4 ] * 100 ) ) );
        ui->setYawValue->setText( QString::number( static_cast< int >( data[ 5 ] * 100 ) ) );
        emit sendDrawingPositions( data[ 0 ], data[ 2 ], data[ 0 ], data[ 1 ] );
        emit sendTrackBallPosition( Eigen::Vector3d{ data[ 0 ], data[ 1 ], data[ 2 ] } );
    }
}
void GSMainWindow::printDeviation( void )
{
    std::vector< double > data = deviationPositionData.getPositionAndVelocity( "current", "position" );
    if( !data.empty() )
    {
        ui->differenceXValue->setText( QString::number( static_cast< int >( data[ 0 ] * 100.0 ) ) );
        ui->differenceYValue->setText( QString::number( static_cast< int >( data[ 1 ] * 100.0 ) ) );
        ui->differenceZValue->setText( QString::number( static_cast< int >( data[ 2 ] * 100.0 ) ) );
        ui->differenceRollValue->setText( QString::number( static_cast< int >( data[ 3 ] * 180.0 / M_PI ) ) );
        ui->differencePitchValue->setText( QString::number( static_cast< int >( data[ 4 ] * 180.0 / M_PI ) ) );
        ui->differenceYawValue->setText( QString::number( static_cast< int >( data[ 5 ] * 180.0 / M_PI ) ) );
        emit sendDrawingPositions( data[ 0 ], data[ 2 ], data[ 0 ], data[ 1 ] );
    }
}

void GSMainWindow::printCurrentPosition( Eigen::VectorXd position, Eigen::VectorXd thrusterAzimuth )
{
    std::vector< double > data = rovPosition.getPositionAndVelocity( "current", "position" );

    if( !data.empty() )
    {
        ui->currentXValue->setText( QString::number( static_cast< int >( data[ 0 ] * 100 ) ) );
        ui->currentYValue->setText( QString::number( static_cast< int >( data[ 1 ] * 100 ) ) );
        ui->currentZValue->setText( QString::number( static_cast< int >( data[ 2 ] * 100 ) ) );
        ui->currentRollValue->setText( QString::number( static_cast< int >( data[ 3 ] * 180 / M_PI ) ) );
        ui->currentPitchValue->setText( QString::number( static_cast< int >( data[ 4 ] * 180 / M_PI ) ) );
        ui->currentYawValue->setText( QString::number( static_cast< int >( data[ 5 ] * 180 / M_PI ) ) );
        emit goCalculateDeviation();
    }
}
void GSMainWindow::receiveSpaceStatus( int status )
{
    if( status == 1 )
        ui->spaceStatusLabel->setStyleSheet( ( "QLabel { background-color : darkGreen;}" ) );
    else
        ui->spaceStatusLabel->setStyleSheet( ( "QLabel { background-color : darkRed;}" ) );
}

void GSMainWindow::receiveCameraStatus( int status )
{

    if( status == 1 )
        ui->cameraStatusLabel->setStyleSheet( ( "QLabel { background-color : darkGreen;}" ) );
    else
        ui->cameraStatusLabel->setStyleSheet( ( "QLabel { background-color : darkRed;}" ) );
}

void GSMainWindow::receiveConnectionStatus( int status )
{
    connectionStatus = status;
    if( status == 1 )
    {
        ui->connectionStatusLabel->setStyleSheet( ( "QLabel { background-color : darkGreen;}" ) );
        ui->connectButton->setText( "Disconnect" );
    }
    else
    {
        ui->connectionStatusLabel->setStyleSheet( ( "QLabel { background-color : darkRed;}" ) );
        ui->connectButton->setText( "Roll out!" );
    }
}

void GSMainWindow::changeCamera( void )
{
    ++cameraChosen;
    if( cameraChosen >= numberOfCams )
    {
        cameraChosen = 0;
    }
    emit sendVideoSetup( cameraChosen );
}

void GSMainWindow::toggleCoralProcessing()
{
    if( coralProcessing == 0u )
    {
        coralProcessing = 1u;
    }
    else
    {
        coralProcessing = 0u;
    }
    emit sendCoralProcessingOnOff( coralProcessing );
}

void GSMainWindow::changeSteeringMode( void )
{
    if( steeringMode )
    {
        steeringMode = 0;
        ui->precisionLabel->setText( "Fast" );
    }
    else
    {
        steeringMode = 1;
        ui->precisionLabel->setText( "Precise" );
    }
}
void GSMainWindow::testModeEnable( int enabled )
{
    ui->motor1Slider->setEnabled( enabled );
    ui->motor2Slider->setEnabled( enabled );
    ui->motor3Slider->setEnabled( enabled );
    ui->motor4Slider->setEnabled( enabled );
    ui->motor5Slider->setEnabled( enabled );
    ui->servo1Slider->setEnabled( enabled );
    ui->servo2Slider->setEnabled( enabled );
    testMode = enabled;
}
void GSMainWindow::updateMotorPWMValues( void )
{
    ui->motor1Value->setText( QString::number( ui->motor1Slider->value() ) );
    ui->motor2Value->setText( QString::number( ui->motor2Slider->value() ) );
    ui->motor3Value->setText( QString::number( ui->motor3Slider->value() ) );
    ui->motor4Value->setText( QString::number( ui->motor4Slider->value() ) );
    ui->motor5Value->setText( QString::number( ui->motor5Slider->value() ) );
    ui->servo1Value->setText( QString::number( ui->servo1Slider->value() ) );
    ui->servo2Value->setText( QString::number( ui->servo2Slider->value() ) );
}
void GSMainWindow::mouseMoveEvent( QMouseEvent* event )
{

    // ui->XTextLabel->setText(QString(event->source()));
    QCursor::setPos( 0, 70 );
}

void GSMainWindow::drawFirstGraphics( void )
{
    ui->orientationLabel->setPixmap( QPixmap::fromImage( QImage( ":/images/resources/images/look.png" ) ) );
}

void GSMainWindow::toggleConnection()
{
    if( connectionStatus == 1 )
    {
        emit openConnection();
    }
    else
    {
        emit closeConnection();
    }
}

void GSMainWindow::parseOpenGripper()
{
    emit openGripper();
}
void GSMainWindow::parseCloseGripper()
{
    emit closeGripper();
}
void GSMainWindow::parseClenchGripper()
{
    emit clenchGripper();
}
void GSMainWindow::parseStretchGripper()
{
    emit stretchGripper();
}
void GSMainWindow::parseOpenGulper()
{
    emit openGulper();
}
void GSMainWindow::parseCloseGulper()
{
    emit closeGulper();
}
void GSMainWindow::parseStopGulper()
{
    emit stopGulper();
}
void GSMainWindow::parseBackwardGulper()
{
    emit backwardGulper();
}
void GSMainWindow::parseForwardGulper()
{
    emit forwardGulper();
}
