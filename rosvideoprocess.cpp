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

    incomingFrame = cv_bridge::toCvCopy(incoming)->image;


    process();
    QImage output((uchar*)processedFrame.data, processedFrame.cols, processedFrame.rows, processedFrame.step, QImage::Format_RGB888);

    emit sendCameraFrame(output);
}

void rosVideoProcess::process(void)
{
    //createMask(incomingFrame);
    //finalFrame=originalFrame;
    //cv::cvtColor(processedFrame,processedFrame,cv::COLOR_RGB2HSV);

}

void rosVideoProcess::changeDetection()
{

}

void rosVideoProcess::createMask(cv::Mat &procImg)
{
    cv::Mat hsvImg;
    cv::cvtColor(procImg,hsvImg,cv::COLOR_RGB2HSV);
    for(int i=0; i<hsvImg.cols;++i)
    {
        for(int j=0; j<hsvImg.rows;++j)
        {

            auto pixl = hsvImg.at<cv::Vec3b>(j,i);
            if( ! ( ( ( pixl[0]>channel1MinChar ) && ( pixl[0]<channel1MaxChar ) ) &&
                    ( ( pixl[1]>channel2MinChar ) && ( pixl[1]<channel2MaxChar ) ) &&
                    ( ( pixl[2]>channel3MinChar ) && ( pixl[2]<channel3MaxChar ) ) ) )
            {
                maskedFrame.at<cv::Vec3b>(j,i) = procImg.at<cv::Vec3b>(j,i);
            }
            else
            {
                maskedFrame.at<cv::Vec3b>(j,i) = 0;
            }
        }
    }
    cv::imshow("dupa",maskedFrame);
}

//void rosVideoProcess::objectTracking()
//{

//}

////void rosVideoProcess::objectDetection()
//{

//}

void rosVideoProcess::transformPicture()
{

}
