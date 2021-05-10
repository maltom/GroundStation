#ifndef ROSVIDEOPROCESS_H
#define ROSVIDEOPROCESS_H

#define numOfCams 2
#include <QObject>
#include <QImage>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/features2d.hpp>
#include <opencv4/opencv2/xfeatures2d.hpp>
#include <opencv4/opencv2/xfeatures2d/nonfree.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

enum class maskType
{
    Binary,
    Color
};
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

constexpr double featureThreshold = 1000.0;
constexpr int octaves = 3;
constexpr int octaveLayers = 3;

using pointsAndFeatures = std::pair<std::vector<cv::KeyPoint>,cv::Mat>;

class rosVideoProcess : public QObject
{
    Q_OBJECT

private:
    cv::Mat incomingFrame = cv::Mat(1920,1080,CV_8UC3);
    cv::Mat oneYearPriorCoral;
    cv::Mat oneYearPriorCoralGrayScale;
    cv::Mat maskedOneYearPriorCoral;
    cv::Mat binaryMaskedOneYearPriorCoral;
    pointsAndFeatures oneYearPriorKeyPointsAndFeatures;
    cv::Mat processedFrame = cv::Mat(1920,1080,CV_8UC3);
    cv::Mat maskedFrame;
    cv::Mat binaryMaskedFrame = cv::Mat(1920,1080,CV_8UC1);
    int statusCamera = 0;
    unsigned int shouldProcess = 0u;
    //bool toggleStream;

    void process(void);
    //void objectDetection();
    void changeDetection();
    cv::Mat cvtToGrayScale(const cv::Mat &inputImg);
    void createMask(cv::Mat& inputImg, cv::Mat& outputImg, maskType type);
    pointsAndFeatures detectKeyPointsAndDescriptors(const cv::Mat& inputImg);
    std::vector< cv::DMatch > matchDescriptors(cv::Mat& descriptors1, cv::Mat& descriptors2);
    //void objectTracking();
    /*
    cv::Mat &firstPicture, cv::Mat &oldPicture,
                              cv::Mat& firstPictureMask, cv::Mat& secondPictureMask*/
    void transformPicture(cv::Mat& inputImg);

public:
    explicit rosVideoProcess(QObject *parent = nullptr);

    ~rosVideoProcess();
signals:
    void sendCameraFrame(QImage finalFrame);
    void sendCameraStatus(int status);
public slots:
    void receiveRosCameraFrame(const sensor_msgs::Image& incoming);
    void receiveCoralProcessingOnOff(unsigned int);
    //void receiveCameraSetup(int device);

};

cv::Mat loadFromQrc(QString qrc, int flag = cv::IMREAD_UNCHANGED);

#endif // ROSVIDEOPROCESS_H
