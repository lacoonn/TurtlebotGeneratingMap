#include "header.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "main");

  ros::NodeHandle sub_nh;
  ros::NodeHandle pub_nh;

  ros::Subscriber subOdom = sub_nh.subscribe("/odom", 100, &odomMsgCallback);
  ros::Subscriber subMap = sub_nh.subscribe("map", 131072, occupancyGrid_Callback);
  ros::Subscriber subRgb = sub_nh.subscribe("/camera/rgb/image_color", 10, &poseMessageReceivedRGB);
  ros::Subscriber subDepth = sub_nh.subscribe("/camera/depth_registered/image_raw", 1000, &poseMessageReceivedDepthRaw);
  ros::Subscriber subScan = sub_nh.subscribe("/scan", 10, &scanMsgCallback);

//ros::Publisher pubGeo = pub_nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1000);

  ros::Rate rate(1000);

  //srand(time(0));

  int limit_distance = distancelimit();

  //while(g_distance <= limit_distance)
  while(1)
  {
    ros::spinOnce();

    rate.sleep();

    std::cout << depthCheck_depth() << std::endl;
  }

  //  depthCheck_depth();
/*
    geometry_msgs::Twist msg;
		msg.linear.x =double(rand())/10;
		msg.angular.z = 3.*double(rand())/double(RAND_MAX)-1.;
    pubGeo.publish(msg);*/


  return 0;
}
