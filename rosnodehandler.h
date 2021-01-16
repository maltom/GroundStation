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

using namespace Eigen;

class rosNodeHandler : public QObject
{
    Q_OBJECT
public:

    explicit rosNodeHandler(QObject *parent = nullptr);

    void sendKToRegulator(const std_msgs::Float32MultiArray& K);
private:

    ros::NodeHandle nodeHandler;
    ros::Publisher rovThrustPublisher;
    ros::Publisher rovAzimPublisher;
    ros::Publisher regulatorToMatlabPublisher;
    ros::Subscriber regulatorReceiver;


signals:

    void sendK(Matrix612 K);

public slots:

    void publishRovParams(VectorXd position, VectorXd thrusterAzimuth);
    void publishABToMatlab(Matrix1212 A, Matrix126 B);
    void update();
};

#endif // ROSNODEHANDLER_H
