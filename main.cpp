#include "header.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "main");

  ros::NodeHandle sub_nh;
  ros::NodeHandle pub_nh;

  g_subOdom = sub_nh.subscribe("/odom", 100, &odomMsgCallback);
  g_subMap = sub_nh.subscribe("map", 131072, occupancyGrid_Callback);
  g_subRgb = sub_nh.subscribe("/camera/rgb/image_color", 10, &poseMessageReceivedRGB);
  g_subDepth = sub_nh.subscribe("/camera/depth_registered/image_raw", 1000, &poseMessageReceivedDepthRaw);
  g_subScan = sub_nh.subscribe("/scan", 10, &scanMsgCallback);

  g_pubGeo = pub_nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1000);

  ros::Rate rate(1000);

  double limit_distance = distancelimit();

  while(g_totaldistance <= limit_distance)
  {
    ros::spinOnce();

    rate.sleep();

    randomMove();

  }

  return 0;
}
