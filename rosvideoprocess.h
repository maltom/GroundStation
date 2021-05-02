#ifndef ROSVIDEOPROCESS_H
#define ROSVIDEOPROCESS_H

#define numOfCams 2
#include <QObject>
#include <QImage>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>


class rosVideoProcess : public QObject
{
    Q_OBJECT

private:
    cv::Mat processedFrame;
    int statusCamera = 0;
    //bool toggleStream;

    void process(void);
    void objectDetection();
    void objectTracking();
    void transformPicture();

public:
    explicit rosVideoProcess(QObject *parent = nullptr);

    ~rosVideoProcess();
signals:
    void sendCameraFrame(QImage finalFrame);
    void sendCameraStatus(int status);
public slots:
    void receiveRosCameraFrame(const sensor_msgs::Image& incoming);
    //void receiveCameraSetup(int device);

};

#endif // ROSVIDEOPROCESS_H
