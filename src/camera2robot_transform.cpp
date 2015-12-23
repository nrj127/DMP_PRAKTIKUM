#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream> // for cout
#include </opt/ros/fuerte/stacks/geometry/tf/include/tf/LinearMath/Matrix3x3.h>
//#include <tf/LinearMath/btMatrix3x3.h>
//#include <geometry_msgs/Vector3.h>

using namespace std;

int main(int argc, char** argv){

    // translation: init marker to robot
    const tf::Vector3 robot_pos(0.1, 0.1, 0.015);
    // rotation: init marker to robot
    const tf::Matrix3x3 robot_rot(1, 0, 0,
                                  0, 1, 0,
                                  0, 0, 1);


    ros::init(argc, argv, "camera2robot_transform");

    ros::NodeHandle nh;

    tf::TransformListener listener;
    static tf::TransformBroadcaster br;

    bool init_transform = false;

    ros::Rate rate(1000.0);

    while (nh.ok()){
      tf::StampedTransform transform;
      tf::StampedTransform out_transform;
      try{
          // if initial transform not set, set it now
          if (!init_transform) {
              // initial transform from camera to robot coordinate system
              ros::Time now = ros::Time::now();
              listener.waitForTransform("/ar_marker_13", "/camera_link", now, ros::Duration(2.0) );
              cout << "Reading tf... ";
              listener.lookupTransform("/ar_marker_13", "/camera_link", now, transform);
              cout << "done" << endl;

              // do transform here

              out_transform.setOrigin(robot_pos);
              out_transform.setBasis(robot_rot);

              // broadcast this transform for rviz evaluation
              cout << "Sending transform: camera > robot... ";
              br.sendTransform(tf::StampedTransform(out_transform, ros::Time::now(), "/ar_marker_13", "/robot"));
              cout << "done" << endl;

              // init_transform = true;
          }

          /*
          ros::Time now = ros::Time::now() ;
          listener.waitForTransform("/ar_marker_9", "/robot", now, ros::Duration(1.0) );
          cout << "Reading tf... ";
          listener.lookupTransform("/ar_marker_9", "/robot", now, transform);
          cout << "done" << endl;

          cout << "Sending transform: obj > robot... ";
          br.sendTransform(tf::StampedTransform(out_transform, ros::Time::now(), "/ar_marker_09", "/object_tracking"));
          cout << "done" << endl;
          */

      }
      catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(0.1).sleep();
      }

    

    rate.sleep();
  }
  return 0;
};

