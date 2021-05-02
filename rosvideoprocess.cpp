#include "rosvideoprocess.h"
#include <opencv2/imgproc/imgproc.hpp>

rosVideoProcess::rosVideoProcess(QObject *parent) : QObject(parent)//, toggleStream(true)
{

}
rosVideoProcess::~rosVideoProcess()
{

}

void rosVideoProcess::receiveRosCameraFrame(const sensor_msgs::Image& incoming)
{

    processedFrame = cv_bridge::toCvCopy(incoming)->image;


    process();
    QImage output((uchar*)processedFrame.data, processedFrame.cols, processedFrame.rows, processedFrame.step, QImage::Format_RGB888);

    emit sendCameraFrame(output);
}

void rosVideoProcess::process(void)
{
    //finalFrame=originalFrame;
    //cv::cvtColor(finalFrame,finalFrame,cv::COLOR_BGR2RGB);

}

void rosVideoProcess::objectTracking()
{

}

void rosVideoProcess::objectDetection()
{

}

void rosVideoProcess::transformPicture()
{

}
