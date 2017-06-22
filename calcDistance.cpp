#include "header.h"

int calcDistance()
{
  nav_msgs::Odometry msg;
  double x1,y1;

  mutex[0].lock(); {
     msg = g_odom;
  } mutex[0].unlock();

  x1 = msg.pose.pose.position.x;
	y1 = msg.pose.pose.position.y;

  double distance = sqrt(pow((x1-g_preX),2.0)+pow((y1-g_preY),2.0));

  g_totaldistance = g_totaldistance + distance;

  g_preX = x1;
  g_preY = y1;

  return 0;
}
