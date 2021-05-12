#ifndef ROSNODEHANDLER_H
#define ROSNODEHANDLER_H

#include "typedefs.h"
#include <QObject>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>

using namespace Eigen;

class rosNodeHandler : public QObject
{
    Q_OBJECT
public:
    explicit rosNodeHandler( QObject* parent = nullptr );

    void sendKToRegulator( const std_msgs::Float32MultiArray& K );
    void sendRosCameraFrame( const sensor_msgs::Image& cameraFrame );

private:
    ros::NodeHandle nodeHandler;
    ros::Publisher rovThrustPublisher;
    ros::Publisher rovAzimPublisher;
    ros::Publisher trackBallPublisher;
    ros::Publisher regulatorToMatlabPublisher;
    ros::Subscriber regulatorReceiver;
    ros::Subscriber mainCameraReceiver;

signals:

    void sendK( Matrix612 K );
    void sendFrameToProcess( const sensor_msgs::Image& cameraFrame );

public slots:

    void publishRovParams( VectorXd position, VectorXd thrusterAzimuth );
    void publishBallPosition( Vector3d Pose );
    void publishABToMatlab( Matrix1212 A, Matrix126 B );
    void update();
};

#endif // ROSNODEHANDLER_H
