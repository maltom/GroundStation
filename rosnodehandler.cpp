#include "rosnodehandler.h"

#include <string>
#include <iostream>
using namespace Eigen;

rosNodeHandler::rosNodeHandler(QObject *parent) : QObject(parent)
{
    // tak naprawdę to Position Publisher
    this->rovThrustPublisher = nodeHandler.advertise<geometry_msgs::Twist>("/gupik/position/rovPose", 1000);
    this->rovAzimPublisher = nodeHandler.advertise<geometry_msgs::Point>("/gupik/position/thrusterPose", 1000);
    //this->regulatorToMatlabPublisher = nodeHandler.advertise<std_msgs::Float32MultiArray>("/turtle1/AB_mat",1000);
    this->trackBallPublisher = nodeHandler.advertise<geometry_msgs::Point>("/gupik/position/trackBall", 1000);
    this->regulatorReceiver = nodeHandler.subscribe("/gupik/position/rov_fdbck",1000,&rosNodeHandler::sendKToRegulator,this);
    this->mainCameraReceiver = nodeHandler.subscribe("/gupik/camera/mainCamera/image_raw",100,&rosNodeHandler::sendRosCameraFrame,this);
}

void rosNodeHandler::publishRovParams(VectorXd position, VectorXd thrusterAzimuth)
{
    geometry_msgs::Twist newPosition;
    if(position.size()>=6)
    {
        newPosition.linear.x = position[0];
        newPosition.linear.y = position[1];
        newPosition.linear.z = position[2];
        newPosition.angular.x = position[3];
        newPosition.angular.y = position[4];
        newPosition.angular.z = position[5];
    }

    geometry_msgs::Point newAzimuth;
    if(thrusterAzimuth.size()>=2)
    {
        newAzimuth.x = thrusterAzimuth[0];
        newAzimuth.y = thrusterAzimuth[1];
        newAzimuth.z = 0.0;
    }

    this->rovAzimPublisher.publish(newAzimuth);
    this->rovThrustPublisher.publish(newPosition);
}

void rosNodeHandler::publishBallPosition(Vector3d Pose)
{
    geometry_msgs::Point msg;
    msg.x = Pose.x();
    msg.y = Pose.y();
    msg.z = Pose.z();
    this->trackBallPublisher.publish(msg);
}

void rosNodeHandler::publishABToMatlab(Matrix1212 A, Matrix126 B)
{
    std_msgs::Float32MultiArray translatedAB;
    translatedAB.layout.dim.emplace_back(std_msgs::MultiArrayDimension());
    translatedAB.layout.dim[0].label = "ABrows";
    translatedAB.layout.dim[0].size = 24;
    translatedAB.layout.dim[0].stride = 12*24;
    translatedAB.layout.dim.emplace_back(std_msgs::MultiArrayDimension());
    translatedAB.layout.dim[1].label = "ABcolumns";
    translatedAB.layout.dim[1].size = 12;
    translatedAB.layout.dim[1].stride = 12;
    for(int i=0;i<12;++i)
    {
        for(int j=0;j<12;++j)
        {
            translatedAB.data.emplace_back(A(i,j));
        }
    }
    for(int i = 0;i<12;++i)
    {
        for(int j=0;j<6;++j)
        {
            translatedAB.data.emplace_back(B(i,j));
        }
        for(int j=6;j<12;++j)
        {
            translatedAB.data.emplace_back(0.0f);
        }
    }
    regulatorToMatlabPublisher.publish(translatedAB);
}

void rosNodeHandler::sendKToRegulator(const std_msgs::Float32MultiArray& K)
{
    Matrix612 translatedK = Matrix612::Zero(6,12);

    for(int i = 0;i<6;++i)
    {
        for(int j = 0;j<12;++j)
        {
            translatedK(i,j) = K.data[i*12+j];
        }
    }

    emit sendK(translatedK);

}

void rosNodeHandler::sendRosCameraFrame(const sensor_msgs::Image& cameraFrame)
{
    emit sendFrameToProcess(cameraFrame);
}

void rosNodeHandler::update()
{
    //ros::Rate loop_rate(10); // too fast loop rate crashed the GUI
    //while (ros::ok())
    //{
    ros::spinOnce();
    //loop_rate.sleep();
    //}
}
