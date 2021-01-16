#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <Eigen/Dense>
#include <QMetaType>

typedef Eigen::Matrix<double,12,12> Matrix1212;
typedef Eigen::Matrix<double,6,12> Matrix612;
typedef Eigen::Matrix<double,12,6> Matrix126;

Q_DECLARE_METATYPE(Matrix1212)
Q_DECLARE_METATYPE(Matrix612)
Q_DECLARE_METATYPE(Matrix126)

class TypeDefs
{
public:
    TypeDefs();
};

#endif // TYPEDEFS_H
