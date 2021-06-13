#ifndef MOSAICPROCESSOR_H
#define MOSAICPROCESSOR_H

#include <QObject>

#include <array>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class mosaicProcessor : public QObject
{
    Q_OBJECT
public:
    static constexpr unsigned numberOfFrames{ 5u };
    explicit mosaicProcessor( QObject* parent = nullptr );
    enum frameNumbers
    {
        front,
        right,
        back,
        left,
        top
    };

private:
    cv::Mat outputImage;
    unsigned frameIndexToReceive{ 0u };
    // void processFrames();

    std::array< cv::Mat, numberOfFrames > incomingFrames;
private slots:

public slots:
    void receiveFrameToProcess( const cv::Mat& incomingFrame );

signals:
    void sendOutputImage( cv::Mat outputImage );
};

#endif // MOZAICPROCESSOR_H
