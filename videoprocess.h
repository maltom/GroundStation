#ifndef VIDEOPROCESS_H
#define VIDEOPROCESS_H

#define numOfCams 2
#include <QObject>
#include <QImage>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class videoProcess : public QObject
{
    Q_OBJECT

private:
    cv::Mat originalFrame;
    cv::Mat finalFrame;
    cv::VideoCapture *cameraPic;
    int statusCamera = 0;
    //bool toggleStream;

    void process(void);
    void objectDetection();
    void objectTracking();
    void transformPicture();

public:
    explicit videoProcess(QObject *parent = nullptr);

    ~videoProcess();

signals:
    void sendCameraFrame(QImage finalFrame);
    void sendCameraStatus(int status);
public slots:
    void receiveCameraFrame(void);
    void receiveCameraSetup(int device);
    //void receiveToggleStream(void);
};

#endif // VIDEOPROCESS_H
