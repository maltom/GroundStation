#include "positiondata.h"
#include <cmath>
#include <iterator>

positionData::positionData(QObject *parent) : QObject(parent)
{

}
void positionData::setPositionAndVelocity(std::string time, std::vector<double> vec)
{
    posAndVel *container = nullptr;

    if(time == "current")
        container = &current;
    else if(time == "future")
        container = &future;
    else if(time == "past")
        container = &past;
    else
        return;

    container->x = vec[0];
    container->y = vec[1];
    container->z = vec[2];
    container->roll = vec[3];
    container->pitch = vec[4];
    container->yaw = vec[5];

    if(vec.size()==12)
    {
        container->xVel = vec[6];
        container->yVel = vec[7];
        container->zVel = vec[8];
        container->rollVel = vec[9];
        container->pitchVel = vec[10];
        container->yawVel = vec[11];
    }
}

void positionData::setPositionAndVelocity(std::string time, double x, double y, double z,
                            double roll, double pitch, double yaw)
{
    posAndVel *container = nullptr;

    if(time == "current")
        container = &current;
    else if(time == "future")
        container = &future;
    else if(time == "past")
        container = &past;
    else
        return;
    container->x = x;
    container->y = y;
    container->z = z;
    container->roll = roll;
    container->pitch = pitch;
    container->yaw = yaw;

}


void positionData::setPositionAndVelocity(std::string time, double x, double y, double z,
                            double roll, double pitch, double yaw,
                            double xVel, double yVel, double zVel,
                            double rollVel, double pitchVel, double yawVel)
{
    posAndVel *container = nullptr;

    if(time == "current")
        container = &current;
    else if(time == "future")
        container = &future;
    else if(time == "past")
        container = &past;
    else
        return;
    container->x = x;
    container->y = y;
    container->z = z;
    container->roll = roll;
    container->pitch = pitch;
    container->yaw = yaw;
    container->xVel = xVel;
    container->yVel = yVel;
    container->zVel = zVel;
    container->rollVel = rollVel;
    container->pitchVel = pitchVel;
    container->yawVel = yawVel;

}

std::vector<double> positionData::getPositionAndVelocity(std::string time, std::string mode)
{
    posAndVel *container = nullptr;
    std::vector<double> data;

    if(time == "current")
        container = &current;
    else if(time == "future")
        container = &future;
    else if(time == "past")
        container = &past;
    else return data;



    if(mode == "position")
    {
        double *point = &container->x;
        for(int i=0;i <6;++i)
        {
            data.push_back(*point);
            point++;
        }
    }
        else if(mode == "velocity")
    {
        double *point = &container->xVel;
        for(int i=0;i <6;++i)
        {
            data.push_back(*point);
            point++;
        }
    }
        else if (mode == "positionAndVelocity")
    {
        double *point = &container->xVel;
        for(int i=0;i <12;++i)
        {
            data.push_back(*point);
            point++;
        }
    }

    return data;
}

void positionData::recalculateSpaceMousePosition(void)
{
    int n=2;
    double *point = &current.x;
    for(int i=0;i <6;++i)
    {
        ((*point < 0) && (n%2==0)) ?
                    (*point = - std::pow(*point/3.5,n)/std::pow(10,n)) :
                    (*point = std::pow(*point/3.5,n)/std::pow(10,n));
        point++;
    }
}
void positionData::calculateStep(std::vector<double> &vec, int steeringMode)
{
    for(auto val = vec.begin(); val!=vec.end(); ++val)
    {
        if(steeringMode==0)
            *val *= fastFactor;
        else
            *val *= preciseFactor;
    }
}


void positionData::addToPositionAndVelocity(std::string time, std::vector<double> vec)
{
    posAndVel *container = nullptr;

    if(time == "current")
        container = &current;
    else if(time == "future")
        container = &future;
    else if(time == "past")
        container = &past;
    else
        return;

    double *point = &container->x;
    for(auto val:vec)
    {
        *point += val;
        ++point;
    }

}
void positionData::addToPositionAndVelocity(std::string time, double x, double y, double z,
                            double roll, double pitch, double yaw)
{
    posAndVel *container = nullptr;

    if(time == "current")
        container = &current;
    else if(time == "future")
        container = &future;
    else if(time == "past")
        container = &past;
    else
        return;

    container->x += x;
    container->y += y;
    container->z += z;
    container->roll += roll;
    container->pitch += pitch;
    container->yaw += yaw;
}

void positionData::addToPositionAndVelocity(std::string time, double x, double y, double z,
                            double roll, double pitch, double yaw,
                            double xVel, double yVel, double zVel,
                            double rollVel, double pitchVel, double yawVel)
{
    posAndVel *container = nullptr;

    if(time == "current")
        container = &current;
    else if(time == "future")
        container = &future;
    else if(time == "past")
        container = &past;
    else
        return;
    container->x += x;
    container->y += y;
    container->z += z;
    container->roll += roll;
    container->pitch += pitch;
    container->yaw += yaw;
    container->xVel += xVel;
    container->yVel += yVel;
    container->zVel += zVel;
    container->rollVel += rollVel;
    container->pitchVel += pitchVel;
    container->yawVel += yawVel;

}
