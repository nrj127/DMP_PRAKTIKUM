#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main()
{

   std::cout << "Project start";
   ros::init(argc,argv,"kinectViewer");
   ros::NodeHandle n;
   ros::Subscriber sub = n.subscribe("/camera/rgb/image_color",1,rgbCallback);
   ros::Subscriber depth = n.subscibe("/camera/depth/image",1,depthCallback);
   ros::spin(); 





    return 0;
		}
