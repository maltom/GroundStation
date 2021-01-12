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

std::vector<double> positionData::getPositionAndVelocity(std::string time, std::string mode) const
{
    const posAndVel *container = nullptr;
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
        const double *point = &container->x;
        for(int i=0;i <6;++i)
        {
            data.push_back(*point);
            point++;
        }
    }
        else if(mode == "velocity")
    {
        const double *point = &container->xVel;
        for(int i=0;i <6;++i)
        {
            data.push_back(*point);
            point++;
        }
    }
        else if (mode == "positionAndVelocity")
    {
        const double *point = &container->xVel;
        for(int i=0;i <12;++i)
        {
            data.push_back(*point);
            point++;
        }
    }

    return data;
}
// function which recalculates ther basic linear range from -350 to 350 to nonlinear from -100 to 100
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
// function which multiplicates step by precision factor
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
void positionData::getDifference(std::string time, positionData &A, std::string timeA, positionData &B, std::string timeB)
{
    posAndVel *containerA = nullptr;
    posAndVel *containerB = nullptr;
    posAndVel *container = nullptr;

    if(time == "current")
        container = &current;
    else if(time == "future")
        container = &future;
    else if(time == "past")
        container = &past;
    else
        return;

    if(timeA == "current")
        containerA = &A.current;
    else if(timeA == "future")
        containerA = &A.future;
    else if(timeA == "past")
        containerA = &A.past;
    else return;

    if(timeB == "current")
        containerB = &B.current;
    else if(timeB == "future")
        containerB = &B.future;
    else if(timeA == "past")
        containerB = &B.past;
    else return;

    container->x = containerA->x - containerB->x;
    container->y = containerA->y - containerB->y;
    container->z = containerA->z - containerB->z;
    container->roll = containerA->roll - containerB->roll;
    container->pitch = containerA->pitch - containerB->pitch;
    container->yaw = containerA->yaw - containerB->yaw;
    container->xVel = containerA->xVel - containerB->xVel;
    container->yVel = containerA->yVel - containerB->yVel;
    container->zVel = containerA->zVel - containerB->zVel;
    container->rollVel = containerA->rollVel - containerB->rollVel;
    container->pitchVel = containerA->pitchVel - containerB->pitchVel;
    container->yawVel = containerA->yawVel - containerB->yawVel;
}

