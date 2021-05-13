#include "rosvideoprocess.h"
#include <algorithm>
#include <iostream>
#include <vector>
#include <QFile>

#include <opencv4/opencv2/calib3d.hpp>
rosVideoProcess::rosVideoProcess( QObject* parent ) : QObject( parent ) //, toggleStream(true)
{
    oneYearPriorCoral = loadFromQrc( ":/images/resources/images/oneYearPrior.png" );
    //    cv::imshow("TEST",oneYearPriorCoral);
    createMask( oneYearPriorCoral, maskedOneYearPriorCoral, maskType::Color );
    createMask( oneYearPriorCoral, binaryMaskedOneYearPriorCoral, maskType::Binary );
    cvtToGrayScale( oneYearPriorCoral, oneYearPriorCoralGrayScale );
    oneYearPriorKeyPointsAndFeatures = detectKeyPointsAndDescriptors( oneYearPriorCoralGrayScale );
    //    cv::imshow("TEST2",maskedOneYearPriorCoral);
}
rosVideoProcess::~rosVideoProcess() {}

void rosVideoProcess::receiveRosCameraFrame( const sensor_msgs::Image& incoming )
{

    incomingFrame = cv_bridge::toCvCopy( incoming )->image;

    process();
    QImage output;
    if( processedFrame.type() == CV_8UC3 )
    {
        output = QImage( ( uchar* )processedFrame.data,
                         processedFrame.cols,
                         processedFrame.rows,
                         processedFrame.step,
                         QImage::Format_RGB888 );
    }
    else
    {
        output = QImage( ( uchar* )processedFrame.data,
                         processedFrame.cols,
                         processedFrame.rows,
                         processedFrame.step,
                         QImage::Format_Grayscale8 );
    }

    emit sendCameraFrame( output );
}

void rosVideoProcess::process( void )
{
    if( shouldProcess == 1u )
    {
        // createMask(incomingFrame,maskedFrame,maskType::Color);
        // finalFrame=originalFrame;
        // cv::cvtColor(processedFrame,processedFrame,cv::COLOR_RGB2HSV);

        transformIncomingPicture( incomingFrame, incomingFrameGray );
        //        processedFrame = transformedFrame;
        changeDetection();
    }
    else
    {
        processedFrame = incomingFrame;
    }
}

void rosVideoProcess::morphOpenClose( const cv::Mat& input, cv::Mat& output, const int strelSize )
{
    cv::Mat strel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size( strelSize, strelSize ) /*,cv::Point(firstStrelDim/2,firstStrelDim/2)*/ );
    cv::Mat intermediate( input.rows, input.cols, input.type() );

    cv::morphologyEx( input, intermediate, cv::MORPH_OPEN, strel );
    cv::morphologyEx( intermediate, output, cv::MORPH_CLOSE, strel );
}

void rosVideoProcess::changeDetection()
{
    createMask( transformedFrame, incomingFrameBinaryMasked, maskType::Binary );
    createMask( transformedFrame, incomingFrameMasked, maskType::Color );

    cv::Mat incomingFrameMorphed(
        incomingFrameBinaryMasked.rows, incomingFrameBinaryMasked.cols, incomingFrameBinaryMasked.type() );
    cv::Mat oneYearPriorMorphed(
        binaryMaskedOneYearPriorCoral.rows, binaryMaskedOneYearPriorCoral.cols, binaryMaskedOneYearPriorCoral.type() );

    morphOpenClose( incomingFrameBinaryMasked, incomingFrameMorphed, firstStrelDim );
    morphOpenClose( binaryMaskedOneYearPriorCoral, oneYearPriorMorphed, firstStrelDim );

    cv::Mat reductionArea;

    cv::subtract( oneYearPriorMorphed, incomingFrameMorphed, reductionArea );

    processedFrame = incomingFrameMorphed;
    cv::imshow( "TEST", oneYearPriorMorphed );
    cv::imshow( "TEST2", incomingFrameMorphed );
    cv::imshow( "TEST3", transformedFrame );
}

void rosVideoProcess::createMask( cv::Mat& inputImg, cv::Mat& outputImg, maskType type )
{
    cv::Mat hsvImg;
    cv::cvtColor( inputImg, hsvImg, cv::COLOR_RGB2HSV );
    /*auto deb = channel1MinChar;
auto deb1 = channel1MaxChar;
auto deb2 = channel2MinChar;
auto deb3 = channel2MaxChar;
auto deb4 = channel3MinChar;
auto deb5 = channel3MaxChar;*/
    if( type == maskType::Color )
    {
        outputImg = cv::Mat( inputImg.rows, inputImg.cols, CV_8UC3 );
    }
    if( type == maskType::Binary )
    {
        outputImg = cv::Mat( inputImg.rows, inputImg.cols, CV_8UC1 );
    }
    for( int i = 0; i < hsvImg.cols; ++i )
    {
        for( int j = 0; j < hsvImg.rows; ++j )
        {

            auto pixl = hsvImg.at< cv::Vec3b >( j, i );
            if( ( ( ( pixl[ 0 ] > channel1MinChar ) && ( pixl[ 0 ] < channel1MaxChar ) )
                  && ( ( pixl[ 1 ] > channel2MinChar ) && ( pixl[ 1 ] < channel2MaxChar ) )
                  && ( ( pixl[ 2 ] > channel3MinChar ) && ( pixl[ 2 ] < channel3MaxChar ) ) ) )
            {
                if( type == maskType::Color )
                {
                    outputImg.at< cv::Vec3b >( j, i ) = inputImg.at< cv::Vec3b >( j, i );
                }
                if( type == maskType::Binary )
                {
                    outputImg.at< unsigned char >( j, i ) = { 255 };
                }
            }
            else
            {
                if( type == maskType::Color )
                {
                    outputImg.at< cv::Vec3b >( j, i ) = { 0, 0, 0 };
                }
                if( type == maskType::Binary )
                {
                    outputImg.at< unsigned char >( j, i ) = { 0 };
                }
            }
        }
    }
    // cv::imshow("TEST",maskedFrame);
}

// void rosVideoProcess::objectTracking()
//{

//}

////void rosVideoProcess::objectDetection()
//{

//}

void rosVideoProcess::cvtToGrayScale( const cv::Mat& inputImg, cv::Mat& inputGrayScaleImg )
{
    cv::cvtColor( inputImg, inputGrayScaleImg, cv::COLOR_RGB2GRAY );
}

// secondInputImg = gray scale img
void rosVideoProcess::transformIncomingPicture( const cv::Mat& inputImg, cv::Mat& secondInputImg )
{
    cvtToGrayScale( inputImg, secondInputImg );
    auto inputKptsFtrs         = detectKeyPointsAndDescriptors( secondInputImg );
    auto indexPairs            = matchDescriptors( oneYearPriorKeyPointsAndFeatures.second, inputKptsFtrs.second );
    auto matchingPointsIndexes = matchingPointsArrays( inputKptsFtrs, oneYearPriorKeyPointsAndFeatures, indexPairs );
    auto estimatedTransform    = cv::estimateAffine2D( matchingPointsIndexes.first, matchingPointsIndexes.second );
    cv::warpAffine( inputImg,
                    transformedFrame,
                    estimatedTransform,
                    { oneYearPriorCoralGrayScale.cols, oneYearPriorCoralGrayScale.rows } );

    cv::imshow( "TEST4", secondInputImg );
    cv::Mat img_matches;
    cv::drawMatches( oneYearPriorCoralGrayScale,
                     oneYearPriorKeyPointsAndFeatures.first,
                     secondInputImg,
                     inputKptsFtrs.first,
                     indexPairs,
                     img_matches );

    cv::imshow( "TEST5", img_matches );
    // trainidx = inputimg queryidx of oneyearprior/original
}

std::vector< cv::DMatch > rosVideoProcess::matchDescriptors( cv::Mat& descriptors1, cv::Mat& descriptors2 )
{
    cv::Ptr< cv::DescriptorMatcher > matcher = cv::DescriptorMatcher::create( cv::DescriptorMatcher::BRUTEFORCE );
    std::vector< cv::DMatch > result;
    matcher->match( descriptors1, descriptors2, result );
    std::sort( result.begin(), result.end(), []( const cv::DMatch& a, const cv::DMatch& b ) -> bool {
        return a.distance < b.distance;
    } );
    if( result.size() > numberOfKeyPointsForAffine )
    {
        result.resize( numberOfKeyPointsForAffine );
    }
    return result;
}

pointsAndFeatures rosVideoProcess::detectKeyPointsAndDescriptors( const cv::Mat& inputImg )
{

    cv::Ptr< cv::xfeatures2d::SURF > detector
        = cv::xfeatures2d::SURF::create( featureThreshold, octaves, octaveLayers );
    pointsAndFeatures kptsFtrs;

    detector->detectAndCompute( inputImg, cv::noArray(), kptsFtrs.first, kptsFtrs.second );
    return kptsFtrs;
}

void rosVideoProcess::receiveCoralProcessingOnOff( unsigned int toggle )
{
    shouldProcess = toggle;
}

matchingKeyPointArrays rosVideoProcess::matchingPointsArrays( const pointsAndFeatures& inputImg,
                                                              const pointsAndFeatures& originalImg,
                                                              const std::vector< cv::DMatch >& matches )
{
    matchingKeyPointArrays result;
    for( auto& in : matches )
    {
        if( ( in.trainIdx < static_cast< const int >( inputImg.first.size() ) )
            && ( in.queryIdx < static_cast< const int >( originalImg.first.size() ) ) )
        {
            result.first.emplace_back( inputImg.first.at( in.trainIdx ).pt );
            result.second.emplace_back( originalImg.first.at( in.queryIdx ).pt );
        }
    }
    return result;
}

cv::Mat loadFromQrc( const QString& qrc, int flag )
{
    // double tic = double(getTickCount());

    QFile file( qrc );
    cv::Mat m;
    if( file.open( QIODevice::ReadOnly ) )
    {
        qint64 sz = file.size();
        std::vector< uchar > buf( sz );
        file.read( ( char* )buf.data(), sz );
        m = cv::imdecode( buf, flag );
    }
    return m;
}
