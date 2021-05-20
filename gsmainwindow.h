#ifndef GSMAINWINDOW_H
#define GSMAINWINDOW_H

// szerokosc 1280 wysokosc 221

#include <QMainWindow>
#include <QThread>
#include <Eigen/Dense>
#include "positiondata.h"
#include "typedefs.h"

QT_BEGIN_NAMESPACE
    namespace Ui
    {
    class GSMainWindow;
    }
QT_END_NAMESPACE

class LQRHandler;

class GSMainWindow : public QMainWindow
{
    Q_OBJECT

private:
    Ui::GSMainWindow* ui;
    // Threads
    QThread* videoThread      = nullptr;
    QThread* spaceMouseThread = nullptr;
    QThread* drawingThread    = nullptr;
    QThread* regulatorThread  = nullptr;
    LQRHandler* regulator     = nullptr;

    // Technical Values
    const int numberOfCams                 = 2;
    static constexpr int regulatorTickTime = 10;
    int x3                                 = 5;
    // Modes
    int steeringMode             = 0; // fast = 0, precise = 1
    int cameraChosen             = 0; // frontal camera = 0, downward camera = 0
    int testMode                 = 0; // test Mode for sending custom PWM values
    unsigned int coralProcessing = 0u;
    // positionData
    positionData spaceMousePositionData; // only current is in use
    positionData deviationPositionData;  // only current is in use
    positionData rovPosition; // past - previous timestep, current - present time step (from STM), future - set position

    // Startup functions
    void videoStart( void );
    void spaceMouseStart( void );
    void modeButtonsInitialization( void );
    void drawingStart( void );
    void regulatorStart( void );
    // Casual functions
    void setTargetPosition( void );
    void calculateDeviation( void );
    // initial graphics
    void drawFirstGraphics( void );
    // mouse events
    void mouseMoveEvent( QMouseEvent* event );

public:
    GSMainWindow( QWidget* parent = nullptr );
    ~GSMainWindow();

public slots:
    void receiveCameraFrame( QImage frame ); // incoming camera view to show
    void receiveCoordinates( int x,
                             int y,
                             int z, // incoming coordinates from Space Mouse manipulator
                             int roll,
                             int pitch,
                             int yaw );
    void receiveSpaceStatus( int status );  // status of space mouse
    void receiveCameraStatus( int status ); // status of camera
    void receiveOrientationDrawing( QImage drawing );

    void changeCamera( void );
    void toggleCoralProcessing();
    void changeSteeringMode( void );

    void printSpaceMouseCoordinates( void );
    void printSetTargetPosition( void );
    void printCurrentPosition( Eigen::VectorXd position, Eigen::VectorXd thrusterAzimuth );
    void printDeviation( void );
    void testModeEnable( int enabled );
    // showing PWM in test mode
    void updateMotorPWMValues( void );

    // drawing events
    // void drawLines(void);
signals:
    void goPrintSpaceMouseCoordinates();
    void goSetTargetPosition();
    void goCalculateDeviation();
    void goPrintSetTargetPosition();
    void goPrintDeviation();
    void sendCoralProcessingOnOff( unsigned int );
    void sendVideoSetup( int device );
    void sendDrawingPositions( double x11, double y11, double x21, double y21 );
    void sendTrackBallPosition( Eigen::Vector3d );

    // void
};
#endif // GSMAINWINDOW_H
