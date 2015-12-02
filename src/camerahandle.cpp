#include "camerahandle.h"


vector<double> CameraHandle::getPos() const
{
    return pos;
}

CameraHandle::CameraHandle()
{
    // init pos vector with 0
    pos.reserve(3);
    std::fill(pos.begin(), pos.end(), 0);
}

void CameraHandle::callback(const geometry_msgs::Twist::ConstPtr &message)
{
    pos[0] = message.get()->linear.x;
    pos[1] = message.get()->linear.y;
    pos[2] = message.get()->linear.z;
}
