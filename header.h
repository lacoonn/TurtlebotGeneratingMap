#ifndef _HEADER_

#define _HEADER_

#include <ros/ros.h>
#include <math.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <tf/tf.h>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <ros/callback_queue.h>
#include <time.h>
#include <iostream>
#include <string>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <turtlebot_actions/TurtlebotMoveAction.h>
#include <actionlib/client/simple_action_client.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_listener.h>
#include <pthread.h>

boost::mutex mutex[5];
//0 - nav_msgs::Odometry msg  /odom
//1 - nav_msgs::OccupancyGrid msg map
//2 - sensor_msgs::Image msg  /camera/rgb/image_color
//3 - sensor_msgs::Image msg  /camera/depth_registered/image_raw
//4 - sensor_msgs::LaserScan g_scan

nav_msgs::Odometry g_odom;
nav_msgs::OccupancyGrid g_map;
sensor_msgs::Image g_rgb;
sensor_msgs::Image g_depth;
sensor_msgs::LaserScan g_scan;
cv::Mat g_display;
int g_totaldistance;
double g_preX;
double g_preY;

ros::Subscriber g_subOdom;
ros::Subscriber g_subMap;
ros::Subscriber g_subRgb;
ros::Subscriber g_subDepth;
ros::Subscriber g_subScan;

ros::Publisher g_pubGeo;

void odomMsgCallback(const nav_msgs::Odometry &msg);
void occupancyGrid_Callback(const nav_msgs::OccupancyGrid &msg);
void poseMessageReceivedRGB(const sensor_msgs::Image& msg);
void poseMessageReceivedDepthRaw(const sensor_msgs::Image& msg);
void scanMsgCallback(const sensor_msgs::LaserScan& msg);
void error_Handle(std::string message);
double distancelimit();
int depthCheck_depth();
int randomMove();
int calc_distance();
void *turtlebotmap(void *data);

void odomMsgCallback(const nav_msgs::Odometry &msg)
{
    mutex[0].lock(); {
        g_odom = msg;
    } mutex[0].unlock();
}
void occupancyGrid_Callback(const nav_msgs::OccupancyGrid &msg)
{
  mutex[1].lock(); {
      g_map = msg;
  } mutex[1].unlock();
}
void poseMessageReceivedRGB(const sensor_msgs::Image& msg)
{
  mutex[2].lock(); {
      g_rgb = msg;
  } mutex[2].unlock();
}
void poseMessageReceivedDepthRaw(const sensor_msgs::Image& msg)
{
  mutex[3].lock(); {
      g_depth = msg;
  } mutex[3].unlock();
}
void scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
    mutex[4].lock(); {
        g_scan = msg;
    } mutex[4].unlock();
}
void error_Handle(std::string message)
{
  std::cout << message << std::endl;
  exit(1);
}

#include "distancelimit.cpp"
#include "gothisAngle.cpp"  //test Angle value;
#include "calcDistance.cpp"
#include "depthCheck.cpp"
#include "turtlebotmap.cpp"
#include "turtlebotMove.cpp"


#endif
