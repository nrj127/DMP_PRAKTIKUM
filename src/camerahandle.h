#ifndef CAMERAHANDLE_H
#define CAMERAHANDLE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>

// TODO which one necessary`
#include <ar_track_alvar/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>



using namespace std;

class CameraHandle
{
    vector<double> pos;
public:
    CameraHandle();
    //void callback(const ar_track_alvar_msgs::AlvarMarkerConstPtr& message);
    vector<double> getPos() const;
};

#endif // CAMERAHANDLE_H
