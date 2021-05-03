#ifndef TYPEDEFS_H
#define TYPEDEFS_H
//#define MATLAB
#include <Eigen/Dense>
#include <QMetaType>
#include <sensor_msgs/Image.h>

typedef Eigen::MatrixXd Matrix1212;
typedef Eigen::MatrixXd Matrix612;
typedef Eigen::MatrixXd Matrix126;

Q_DECLARE_METATYPE(Matrix1212)
Q_DECLARE_METATYPE(Eigen::VectorXd)
Q_DECLARE_METATYPE(Eigen::Vector3d)
Q_DECLARE_METATYPE(sensor_msgs::Image)
//Q_DECLARE_METATYPE(Matrix612)
//Q_DECLARE_METATYPE(Matrix126)

class TypeDefs
{
public:
    TypeDefs();
};

#endif // TYPEDEFS_H
