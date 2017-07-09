#include "header.h"

double dothisAngle()
{
/*
  double rand_num;
  int plusminus;

  rand_num = rand()%314;
  rand_num = rand_num/100;

  plusminus = rand()%2;

  if(plusminus == 0)
  {
    rand_num = rand_num * (-1);
  }

  return rand_num;
*/
	double x, y, radian;
	SelectDirection(x, y, radian);
	std::cout << "Direction => X: " << x << ", Y: " << y << std::endl;

  	return radian;

}
