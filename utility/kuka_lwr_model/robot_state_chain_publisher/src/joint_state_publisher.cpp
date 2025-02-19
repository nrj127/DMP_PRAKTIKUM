// A node to solve the inverse kinematic and publish the desired
// joint angles on a ros topic
//
// Author: Matteo Saveriano

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

// kdl specific include
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <math.h>
#include <fstream>

#define JOINT_NUM 7

#ifndef PI
#define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / PI )
#endif

using namespace std;
using namespace ros;
using namespace KDL;

ChainIkSolverVel_pinv *IKSolv_;
ChainFkSolverPos_recursive *FKSolv_;
JntArray qOld, qIn, qDotOut;
JntArray qMax, qDotMax;
Twist vIn;
Publisher pub_;
Subscriber sub_;
sensor_msgs::JointState jointStateMsg;
int jointNum, seq;
double deltaT;
int use_pos_limit, use_vel_limit;

ChainJntToJacSolver *jnt2jac;
Jacobian jac;
SVD_HH *svd;
std::vector<JntArray> U;
JntArray S;
std::vector<JntArray> V;
JntArray tmp;

void savePosToFile(string fileName, double xPos, double yPos, double zPos)
{
    std::ofstream out_;

    out_.open(fileName.c_str(),ios::app);
    out_ << xPos << "\t" << yPos << "\t"<< zPos << std::endl;
    out_.close();

    return;
}

void inverseKinematicReduntant(const JntArray q_in,
                               const Twist v_in,
                               const Twist posErr,
                               JntArray &qdot_out,
                               double eps=0.00001,
                               int maxiter=150)
{
    //Let the ChainJntToJacSolver calculate the jacobian "jac" for
    //the current joint positions "q_in"
    jnt2jac->JntToJac(q_in,jac);


    Eigen::MatrixXd m(6,7);
    // Free orientation
    unsigned int i,j;
    for (i=0;i<jac.columns();i++)
    {
        for (j=0;j<jac.rows();j++)
        {
            if(j<3)
                m(j,i) = jac(j,i);
            else
            {
                m(j,i) = 0.0;
               // jac(j,i) = 0.0;
            }
        }
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svdMat(m, Eigen::ComputeThinU | Eigen::ComputeThinV);

    //Do a singular value decomposition of "jac" with maximum
    //iterations "maxiter", put the results in "U", "S" and "V"
    //jac = U*S*Vt
   //svd->calculate(jac,U,S,V,maxiter);

    double sum;

    // We have to calculate qdot_out = jac_pinv*v_in
    // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
    // qdot_out = V*S_pinv*Ut*v_in
    // first we calculate Ut*v_in
    for (i=0;i<(unsigned int)svdMat.matrixU().cols();i++)
    {
        sum = 0.0;
        for (j=0;j<(unsigned int)svdMat.matrixU().rows();j++)
        {
            sum+= svdMat.matrixU()(j,i)*(v_in(j)+posErr(j));
        }
        //If the singular value is too small (<eps), don't invert it but
        //set the inverted singular value to zero (truncated svd)
        tmp(i) = sum*(fabs(svdMat.singularValues()(i))<eps?0.0:1.0/svdMat.singularValues()(i));
    }

    //tmp is now: tmp=S_pinv*Ut*(v_in+err), we still have to premultiply
    //it with V to get qdot_out
    for (i=0;i<jac.columns();i++) {
        sum = 0.0;
        for (j=0;j<jac.columns()-4;j++) {
            sum+=svdMat.matrixV()(i,j)*tmp(j);
        }
        //Put the result in qdot_out
        qdot_out(i)=sum;
    }

    return;
}

bool spin_;
void referenceCallback(const nav_msgs::OdometryConstPtr& msgPtr)
{
    spin_ = true;
    size_t i = 0;
    seq++;

    vIn.vel(0) = msgPtr->twist.twist.linear.x;
    vIn.vel(1) = msgPtr->twist.twist.linear.y;
    vIn.vel(2) = msgPtr->twist.twist.linear.z;

    vIn.rot(0) = 0.0;//msgPtr->twist.twist.angular.x;
    vIn.rot(1) = 0.0;//msgPtr->twist.twist.angular.y;
    vIn.rot(2) = 0.0;//msgPtr->twist.twist.angular.z;

    Frame initFrame;
    FKSolv_->JntToCart(qIn, initFrame);

    Twist posErr;
    double gain_  = 50.0;
    posErr.vel(0) = gain_*(msgPtr->pose.pose.position.x-initFrame.p(0));
    posErr.vel(1) = gain_*(msgPtr->pose.pose.position.y-initFrame.p(1));
    posErr.vel(2) = gain_*(msgPtr->pose.pose.position.z-initFrame.p(2));
    posErr.rot(0) = 0.0;//msgPtr->twist.twist.angular.x;
    posErr.rot(1) = 0.0;//msgPtr->twist.twist.angular.y;
    posErr.rot(2) = 0.0;//msgPtr->twist.twist.angular.z;

    inverseKinematicReduntant(qIn, vIn, posErr, qDotOut);

    jointStateMsg.header.stamp = Time::now();
    jointStateMsg.header.seq   = seq;

   // qOld = qIn;

    // Joint velocity  limits
    if(use_vel_limit)
    {
        for(i=0; i<qDotOut.rows(); ++i)
        {
            if(abs(qDotOut(i))>qDotMax(i))
            {
                ROS_WARN("Joint velocity limit %i exceeded", i);
                qDotOut(i) = sign(qDotOut(i))*qDotMax(i);
            }
        }
    }

    qIn.data += deltaT*qDotOut.data;

    // Joint position limits
    if(use_pos_limit)
    {
        for(i=0; i<qIn.rows(); ++i)
        {
            if(abs(qIn(i)>qMax(i)))
            {
                ROS_WARN("Joint limit %i exceeded", i);
                qIn(i) = sign(qIn(i))*qMax(i);
            }
        }
    }

    jointStateMsg.position.assign(qIn.data.data(), qIn.data.data()+qIn.rows());
   // std::copy(, &[0]);
    pub_.publish(jointStateMsg);

    savePosToFile("/home/hwadong/cotesys-lwr4+/kuka_lwr_model/robot_state_chain_publisher/sim2_refPosition.txt",
                  msgPtr->pose.pose.position.x,
                  msgPtr->pose.pose.position.y,
                  msgPtr->pose.pose.position.z);

    savePosToFile("/home/hwadong/cotesys-lwr4+/kuka_lwr_model/robot_state_chain_publisher/sim2_realPosition.txt",
                  initFrame.p(0),
                  initFrame.p(1),
                  initFrame.p(2));


    // Check collision
    FKSolv_->JntToCart(qIn, initFrame);

    Vector temp;
    temp(0) = msgPtr->twist.twist.angular.x,
    temp(1) = msgPtr->twist.twist.angular.y,
    temp(2) = msgPtr->twist.twist.angular.z;
    temp -= initFrame.p;

    double dist_ = temp.Norm() - 0.08;
    if(dist_<0.0)
    {
        ROS_ERROR("COLLISION");
        cout << dist_ << endl;
        exit(1);
    }
}

// ----------------------------------
// ----- MAIN -----------------------
// ----------------------------------
int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "joint_state_publisher");
  NodeHandle node("~");
  ros::Rate wait_rate(100.0);

  // Set joint limits
  qMax.data.resize(JOINT_NUM);
  qMax.data << RAD(170.0), RAD(120.0), RAD(170.0),
                    RAD(120.0), RAD(170.0), RAD(120.0), RAD(170.0);

  qDotMax.data.resize(JOINT_NUM);
  qDotMax.data << RAD(100.0), RAD(110.0), RAD(100.0),
                    RAD(130.0), RAD(130.0), RAD(180.0), RAD(180.0);

  // Initial position
  qIn.data.resize(JOINT_NUM);
  float q_init[JOINT_NUM];

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

  qDotOut.resize(JOINT_NUM);

  if(!node.getParam("use_pos_limit", use_pos_limit))
  {
      use_pos_limit = 1;
  }
  if(!node.getParam("use_vel_limit", use_vel_limit))
  {
      use_vel_limit = 1;
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
  deltaT = 0.001;

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

  IKSolv_ = new ChainIkSolverVel_pinv(chain);
  FKSolv_ = new ChainFkSolverPos_recursive(chain);
  Frame initFrame;
  FKSolv_->JntToCart(qIn, initFrame);

  //Rotation r = initFrame.M;
  //Vector p   = initFrame.p;

  jnt2jac = new ChainJntToJacSolver(chain);
  jac.resize(chain.getNrOfJoints());
  svd = new SVD_HH(jac);
  U.resize(6,JntArray(chain.getNrOfJoints()));
  S(chain.getNrOfJoints());
  V.resize(chain.getNrOfJoints(),JntArray(chain.getNrOfJoints()));
  tmp.resize(chain.getNrOfJoints());

  jnt2jac->JntToJac(qIn,jac);

  string pose_topic_;
  if(!node.getParam ("desired_pose_topic", pose_topic_))
  {
    pose_topic_ = "/reference_state";
  }
  sub_ = node.subscribe <nav_msgs::Odometry> (pose_topic_.c_str(), 1, referenceCallback);
  pub_ = node.advertise <sensor_msgs::JointState> ("/joint_states", 1);

  for(size_t j=0; j<qIn.rows(); ++j)
      jointStateMsg.position[j] = qIn(j);

  spin_ = false;
  while(!spin_)
  {
      pub_.publish(jointStateMsg);
      ros::spinOnce();
      wait_rate.sleep();
  }

  ros::spin();

  return 0;
}
