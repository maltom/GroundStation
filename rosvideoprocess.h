#ifndef ROSVIDEOPROCESS_H
#define ROSVIDEOPROCESS_H

#define numOfCams 2
#include <QObject>
#include <QImage>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/hal/interface.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>


constexpr double channel1Min = 0.509;
constexpr double channel1Max = 0.912;

constexpr double channel2Min = 0.000;
constexpr double channel2Max = 0.800;

constexpr double channel3Min = 0.636;
constexpr double channel3Max = 1.000;

constexpr uchar channel1MinChar = channel1Min * 255;
constexpr uchar channel1MaxChar = 0.912 * 255;

constexpr uchar channel2MinChar = 0.000 * 255;
constexpr uchar channel2MaxChar = 0.800 * 255;

constexpr uchar channel3MinChar = 0.636 * 255;
constexpr uchar channel3MaxChar = 1.000 * 255;

class rosVideoProcess : public QObject
{
    Q_OBJECT

private:
    cv::Mat incomingFrame;
    cv::Mat processedFrame;
    cv::Mat maskedFrame;
    int statusCamera = 0;
    //bool toggleStream;

    void process(void);
    //void objectDetection();
    void changeDetection();
    void createMask(cv::Mat& procImg);
    //void objectTracking();
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
