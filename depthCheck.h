#include "header.h"

#define maxDepth 4096
#define minDepth 0

int depthCheck_depth()
{
  sensor_msgs::Image msg;

  mutex[3].lock(); {
     msg = g_depth;
  } mutex[3].unlock();

  unsigned short len = maxDepth-minDepth;
  int nTotal = msg.height*msg.width;

  unsigned short *pSrc = (unsigned short*) msg.data;

  for(int i=0; i<nTotal; i++, pSrc++)
  {
    if(pSrc[2] == 0.)
    {
      
    }
    else
    {
      int v = (int) (255*6*(1. - ((double) (maxDepth-pSrc[2]))/len));
      if (v < 0)
      v = 0;
      int lb = v & 0xff;

      switch (v / 256)
      {
        case 0:
        printf("%d\n",v);
        break;
        case 1:
        printf("%d\n",v);
        break;
        case 2:
        printf("%d\n",v);
        break;
        case 3:
        printf("%d\n",v);
        break;
        case 4:
        printf("%d\n",v);
        break;
        case 5:
        printf("%d\n",v);
        break;
        default:
        printf("%d\n",v);
        break;
      }
    }
  }

  return 0;
}
