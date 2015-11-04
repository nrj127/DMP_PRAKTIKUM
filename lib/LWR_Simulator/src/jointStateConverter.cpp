// A node to convert and republish the msg <fri_ros_interface/LWR_joints.h> (published by FriRosInterface) 
// into a <sensor_msgs/JointState.h> (read by joint_state_listener.cpp)
//
// Author: Matteo Saveriano

#include <ros/ros.h>
#include <fri_ros_interface/LWR_joints.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

// kdl specific include
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <math.h>
#include <fstream>

#ifndef JOINT_NUM
#define JOINT_NUM 7
#endif

#ifndef PI
#define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A)	((A) * PI / 180.0 )
#endif

using namespace std;
using namespace ros;
using namespace KDL;

Publisher pub_;
Subscriber sub_;
sensor_msgs::JointState jointStateMsg;
int jointNum, seq;

void savePosToFile(string fileName, double xPos, double yPos, double zPos)
{
    std::ofstream out_;

    out_.open(fileName.c_str(),ios::app);
    out_ << xPos << "\t" << yPos << "\t"<< zPos << std::endl;
    out_.close();

    return;
}


bool spin_;
void measuredJointCallback(const fri_ros_interface::LWR_jointsConstPtr& jointPtr)
{
    spin_ = true;
    jointStateMsg.header.stamp = Time::now();
    jointStateMsg.header.seq   = seq++;

    jointStateMsg.position.assign( jointPtr->LWR_joint_array.elems,
                                   jointPtr->LWR_joint_array.elems+jointPtr->LWR_joint_array.size() );
    pub_.publish(jointStateMsg);
}

// ----------------------------------
// ----- MAIN -----------------------
// ----------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "jointStateConverter");
    NodeHandle node("~");
    ros::Rate wait_rate(100.0);

    // Initial position
    JntArray qIn(JOINT_NUM);

    XmlRpc::XmlRpcValue angle_list;
    if(!node.getParam("initial_angles", angle_list)) 
    {
        ROS_INFO("No initial_angles defined. Using default values");
        for (int32_t i = 0; i < JOINT_NUM; ++i) 
            qIn.data(i) = 0.0; // default values
    }
    else
    {
        if(angle_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("initial_angles param should be a list.");
            return false;
        }
        for(int i = 0; i < angle_list.size(); i++)
        {
            if(angle_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("initial_angles entry %d is not of type double.", i);
                return false;
            }
            qIn.data(i) = RAD(static_cast<double>(angle_list[i]));
            ROS_DEBUG("Adding initial joint angle %d = %d", i, qIn.data(i));
        }
    } 

    // gets the location of the robot description on the parameter server
    string full_param_name;
    node.searchParam("robot_description",full_param_name);
    string robot_desc;

    // constructs a kdl tree from the robot model
    node.param(full_param_name, robot_desc, string());
    Tree tree;
    if (!kdl_parser::treeFromString(robot_desc, tree)){
        ROS_ERROR("Failed to extract kdl tree from xml robot description");
        return -1;
    }

    // Get robot segments
    map<string,TreeElement>::const_iterator root = tree.getRootSegment();

    Chain chain;
    tree.getChain(root->first,"kimp_right_arm_7_link", chain);

    // Select only the robot's joints
    jointNum = 0;
    seq = 0;

    Segment s_;
    Joint j_;
    for(size_t i=0; i<chain.getNrOfSegments(); ++i)
    {
        s_ = chain.getSegment(i);
        j_ = s_.getJoint();
        if(j_.getType() != Joint::None)
        {
            jointStateMsg.name.push_back(j_.getName());
            jointStateMsg.position.push_back(0.0);
            jointNum++;
        }
    }

    string joint_topic_;
    if(!node.getParam ("measured_joint_topic", joint_topic_))
    {
        joint_topic_ = "/dhri_lwr/Visualization/Joints/joint_measured_pos";
    }

    std::cout <<  joint_topic_ << std::endl;

    sub_ = node.subscribe <fri_ros_interface::LWR_joints> (joint_topic_.c_str(), 1, measuredJointCallback);
    pub_ = node.advertise <sensor_msgs::JointState> ("/joint_states", 1);

    for(size_t j=0; j<qIn.rows(); ++j)
        jointStateMsg.position[j] = qIn(j);

    spin_ = false;
    //while(!spin_)
    {
        pub_.publish(jointStateMsg);
        //ros::spinOnce();
       // wait_rate.sleep();
    }

    ros::spin();

    return 0;
}
