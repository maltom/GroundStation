#include "videoprocess.h"
#include <opencv2/imgproc/imgproc.hpp>

videoProcess::videoProcess( QObject* parent ) : QObject( parent ) //, toggleStream(true)
{
    cameraPic = new cv::VideoCapture();
}
videoProcess::~videoProcess()
{
    if( cameraPic->isOpened() )
        cameraPic->release();
    delete cameraPic;
}

void videoProcess::receiveCameraFrame( void )
{
    //    if(!toggleStream)
    //        return;
    cameraPic->read( originalFrame );
    if( originalFrame.empty() )
        return;

    process();
    QImage output( ( uchar* )originalFrame.data,
                   originalFrame.cols,
                   originalFrame.rows,
                   originalFrame.step,
                   QImage::Format_RGB888 );

    emit sendCameraFrame( output );
}

void videoProcess::process( void )
{
    finalFrame = originalFrame;
    cv::cvtColor( finalFrame, finalFrame, cv::COLOR_BGR2RGB );
}

void videoProcess::receiveCameraSetup( int device )
{
    if( cameraPic->isOpened() )
        cameraPic->release();
    if( device == 0 )
        cameraPic->open( 0 );
    if( device == 1 )
        // cameraPic->open("http://192.168.0.13/video.mjpg");
        cameraPic->open( "rtsp://admin:admin@169.254.69.70/1234" );

    if( !cameraPic->isOpened() )
    {
        statusCamera = 0;
        emit sendCameraStatus( statusCamera );
        return;
    }
    else
    {
        statusCamera = 1;
        emit sendCameraStatus( statusCamera );
    }
}

void videoProcess::objectTracking() {}

void videoProcess::objectDetection() {}

void videoProcess::transformPicture() {}
