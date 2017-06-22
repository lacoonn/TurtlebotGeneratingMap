#include "header.h"

#define ANGLE_COUNT 200

int depthCheck_depth()  //1 is block , 0 is nonblock
{
  int blockandnonblock=0; //0: block, 1:nonblock
  sensor_msgs::LaserScan msg;

  mutex[4].lock(); {
     msg = g_scan;
  } mutex[4].unlock();

  int size_ranges_index = msg.ranges.size();
  int size_ranges_index_divide = size_ranges_index/2;

  for(int i = 0;i<ANGLE_COUNT;i++)
  {
    if(msg.ranges[size_ranges_index_divide+i]<1)
    {
      blockandnonblock = 1;
    }
  }

  for(int i = 0;i<ANGLE_COUNT;i++)
  {
    if(msg.ranges[size_ranges_index_divide-i]<1)
    {
      blockandnonblock = 1;
    }
  }

  return blockandnonblock;
}
