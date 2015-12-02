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

void CameraHandle::callback(const ar_track_alvar_msgs::AlvarMarkerConstPtr& message)
{
    pos[0] = message.get()->pose.pose.position.x;
    pos[1] = message.get()->pose.pose.position.y;
    pos[2] = message.get()->pose.pose.position.z;
}
