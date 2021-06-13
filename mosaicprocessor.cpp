#include "mosaicprocessor.h"

mosaicProcessor::mosaicProcessor( QObject* parent ) : QObject( parent ) {}

void mosaicProcessor::receiveFrameToProcess( const cv::Mat& incomingFrame )
{
    this->incomingFrames[ frameIndexToReceive++ ] = incomingFrame;
    if( frameIndexToReceive >= numberOfFrames )
    {
        frameIndexToReceive = 0;
    }
}

// void mosaicProcessor::processFrames() {}
