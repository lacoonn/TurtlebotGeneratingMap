#include "header.h"

using namespace std;

void SelectDirection(&x, &y, &radian)
{
	nav_msgs::OccupancyGrid grid;
	nav_msgs::Odometry odom;
	mutex[0].lock(); {
	odom = g_odom;
	} mutex[0].unlock();
	mutex[2].lock(); {
	grid = g_grid;
	} mutex[2].unlock();


  	int width = grid.info.width;
  	int height = grid.info.height;
  	int map_origin_x = grid.info.origin.position.x;
  	int map_origin_y = grid.info.origin.position.y;
  	double map_origin_angle = atan2(2 * grid.info.origin.orientation.w * grid.info.origin.orientation.z, 1 - 2 * pow(grid.info.origin.orientation.z, 2));

	/*
	* 현재 위치 X, Y를 원점을 기준으로 radian 만큼 회전했을 때의 좌표 (The position when the current position X, Y is rotated by radian with respect to the origin)
  	* mX = X * cos(radian) - Y * sin(radian)
	* mY = X * sin(radian) + Y * cos(radian)
  	*/
	// 지도상에서 Turtlebot의 위치
  	int turtlebot_x = (g_odom.pose.pose.position.x - map_origin_x) / g_grid.info.resolution * cos(map_origin_angle) - (g_odom.pose.pose.position.y - map_origin_y) / g_grid.info.resolution * sin(map_origin_angle);
  	int turtlebot_y = (g_odom.pose.pose.position.x - map_origin_x) / g_grid.info.resolution * sin(map_origin_angle) + (g_odom.pose.pose.position.y - map_origin_y) / g_grid.info.resolution * cos(map_origin_angle);
	//cout << "odom.position.x: " << setw(2) << setfill('0') << odom.pose.pose.position.x << ", odom.position.y: " << setw(2) << setfill('0') << odom.pose.pose.position.y << endl;

	double TurtlebotSize = 0.4;
	int turtlebot_size_in_map = (int)(0.4 / g_grid.info.resolution);

  	// 일차원 배열인 Gmapping 맵을 이차원 배열로 변환한다
	int tempMap[height][width];
  	for(int y = 0; y < height; y++) {
    	for(int x = 0; x < width; x++) {
      		int index = width * y + x;
      		if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
				// 지도에서 "> 1"은 장애물
				// 지도에서 "= 0"은 탐색한 공간
				// 지도에서 "< 0"은 아직 탐색하지 않은 공간
				tempMap[y][x] = grid.data[index];
      		}
    	}
  	}
	// 2차원 배열로 변경한 Map에서 n pixel씩 sliding하며 값을 계산한다
	int unitPixel = 3
	int start_x = turtlebot_x - turtlebot_size_in_map;
	int start_y = turtlebot_y - turtlebot_size_in_map;
	if(start_x < 0)
		start_x = 0;
	if(start_y < 0)
		start_y = 0;
	for(start_y; start_y + turtlebot_size_in_map < height; start_y += unitPixel) {
    	for(start_x = 0; start_x + turtlebot_size_in_map < width; start_x += unitPixel) {
			// 값을 계산하기 위해 한 변이 turtlebot_size_in_map인 정사각형을 순회한다
			for(int y = start_y; y < start_y + turtlebot_size_in_map && y < height; y++) {
				for(int x = start_x; x < start_x + turtlebot_size_in_map && y < width; x++) {
					// 지도에서 "> 1"은 장애물
					// 지도에서 "= 0"은 탐색한 공간
					// 지도에서 "< 0"은 아직 탐색하지 않은 공간
					if(tempMap > 1)

				}
			}
    	}
  	}
  	/*
  	* 위치 x, y에서 length만큼 radian 각도로 이동한 위치
  	* point x = cos(Rad) * length
  	* point y = sin(Rad) * length
  	*/
}
