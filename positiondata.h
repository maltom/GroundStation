#ifndef POSITIONDATA_H
#define POSITIONDATA_H

#include <QObject>

class positionData : public QObject
{
    Q_OBJECT
public:
    explicit positionData(QObject *parent = nullptr);

    // set Position overloads
    void setPositionAndVelocity(std::string time, std::vector<double> vec);
    void setPositionAndVelocity(std::string time, double x, double y, double z,
                                double roll, double pitch, double yaw);

    void setPositionAndVelocity(std::string time, double x, double y, double z,
                                double roll, double pitch, double yaw,
                                double xVel, double yVel, double zVel,
                                double rollVel, double pitchVel, double yawVel);

    // get Position
    std::vector<double> getPositionAndVelocity(std::string time, std::string mode);

    // important function to change Space Mouse steering scope from <-350, 350> linear to <-100, 100> nonlinear function
    void recalculateSpaceMousePosition(void);
    void calculateStep(std::vector<double> &vec, int steeringMode);

    // add to Position oveloads
    void addToPositionAndVelocity(std::string time, std::vector<double> vec);
    void addToPositionAndVelocity(std::string time, double x, double y, double z,
                                double roll, double pitch, double yaw);

    void addToPositionAndVelocity(std::string time, double x, double y, double z,
                                double roll, double pitch, double yaw,
                                double xVel, double yVel, double zVel,
                                double rollVel, double pitchVel, double yawVel);
private:
                            //  12345
    double preciseFactor    = 0.000005;
    double fastFactor       = 0.00005;

    struct posAndVel    //don't change the order of elements in the structure
    {
        double x = 0;
        double y = 0;
        double z = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;

        double xVel = 0;
        double yVel = 0;
        double zVel = 0;
        double rollVel = 0;
        double pitchVel = 0;
        double yawVel = 0;
    } current, past, future;



signals:

};

#endif // POSITIONDATA_H
