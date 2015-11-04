#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

int main (int argc, char** argv)
{
    ros::init(argc, argv, "drawGoalPosition");
    ros::NodeHandle node("~");

    ros::Publisher goal_pub = node.advertise<visualization_msgs::Marker>( "/goal_markers", 1 );
    visualization_msgs::Marker marker_goal;
    // Create sample marker
    if( !node.getParam("world_frame", marker_goal.header.frame_id) )
    {
	marker_goal.header.frame_id = "/world";
    }
    marker_goal.header.stamp = ros::Time();
   // marker_goal.id = 1;
    marker_goal.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_goal.action = visualization_msgs::Marker::ADD;
    marker_goal.pose.position.x = 0.0;
    marker_goal.pose.position.y = 0.0;
    marker_goal.pose.position.z = 0.0;
    marker_goal.pose.orientation.x = 0.0;
    marker_goal.pose.orientation.y = 0.0;
    marker_goal.pose.orientation.z = 0.0;
    marker_goal.pose.orientation.w = 1.0;
    marker_goal.color.a = 1.0;
    marker_goal.color.r = 1.0f;
    marker_goal.color.g = 0.75f;
    marker_goal.color.b = 0.0f;
    marker_goal.scale.x = 0.03;
    marker_goal.scale.y = 0.03;
    marker_goal.scale.z = 0.03;

    geometry_msgs::Point pos_;
    std_msgs::ColorRGBA color_;
    color_.a=1.0; color_.r=0.0f; color_.g=1.0f; color_.b=1.0f;
    // Load goal positions from file
    string goal_file;
    if( !node.getParam("goal_file", goal_file) )
    {
	goal_file = "goals.txt";
    }
    string line;
    ifstream myfile (goal_file.c_str());
    if ( myfile.is_open() )
    {
        while ( getline(myfile, line) )
        {
            std::istringstream iss(line);
            // Each line contains x, y, z values
            if ( !(iss >> pos_.x >> pos_.y >> pos_.z) ) 
	    { 
		ROS_ERROR("File corrupted \n");
		break; 
	    } 

            // Fill marker list 
	    marker_goal.points.push_back(pos_);
  	    marker_goal.colors.push_back(color_);
        }
        myfile.close();
    }
    else
    {
        ROS_ERROR("Unable to open file %s \n", goal_file.c_str());
	exit(1);
    } 

    // Load obstacles positions from file
    color_.a=1.0; color_.r=1.0f; color_.g=0.75f; color_.b=0.0f;
    if( !node.getParam("obstacle_file", goal_file) )
    {
	goal_file = "obstacles.txt";
    }
    ifstream myfile2 (goal_file.c_str());
    if ( myfile2.is_open() )
    {
        while ( getline(myfile2, line) )
        {
            std::istringstream iss(line);
            // Each line contains x, y, z values
            if ( !(iss >> pos_.x >> pos_.y >> pos_.z) ) 
	    { 
		ROS_ERROR("File corrupted \n");
		break; 
	    } 

            // Fill marker list 
	    marker_goal.points.push_back(pos_);
  	    marker_goal.colors.push_back(color_);
        }
        myfile2.close();
    }

    double pub_freq;
    if( !node.getParam("publish_frequency", pub_freq) )
	pub_freq = 30.0;

    ros::Rate loop_(pub_freq);
    // Publish goal markers
    while(ros::ok())
    {
        goal_pub.publish(marker_goal);
        loop_.sleep();
    }

    return 0;
}
