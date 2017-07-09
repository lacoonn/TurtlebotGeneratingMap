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

#define ANGLE_COUNT 200

using namespace std;

typedef struct directionValue {
	int value;
	int dX;
	int dY;
} directionValue;

boost::mutex mutex[5];
//0 - nav_msgs::Odometry msg  /odom
//1 - nav_msgs::OccupancyGrid msg map
//2 - sensor_msgs::Image msg  /camera/rgb/image_color
//3 - sensor_msgs::Image msg  /camera/depth_registered/image_raw
//4 - sensor_msgs::LaserScan g_scan


bool isKill = false;
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
void SelectDirection(double &returnX, double &returnY, double &returnRadian);
int turtlebotMove();
double dothisAngle();

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

void handler(int a)
{
	isKill = true;
	//exit(1);
}

int main (int argc, char **argv)
{
	g_totaldistance = 0;

  	ros::init(argc, argv, "main");

  	ros::NodeHandle sub_nh;
  	ros::NodeHandle pub_nh;

  	g_subOdom = sub_nh.subscribe("/odom", 100, &odomMsgCallback);
  	g_subMap = sub_nh.subscribe("map", 131072, occupancyGrid_Callback);
  	g_subRgb = sub_nh.subscribe("/camera/rgb/image_color", 10, &poseMessageReceivedRGB);
  	g_subDepth = sub_nh.subscribe("/camera/depth_registered/image_raw", 1000, &poseMessageReceivedDepthRaw);
  	g_subScan = sub_nh.subscribe("/scan", 10, &scanMsgCallback);

  	g_pubGeo = pub_nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1000);

  	ros::Rate rate(100);
    srand(time(NULL));
  	//double limit_distance = distancelimit();
  	pthread_t p_thread;
  	int thr_id;
  	int status;
  	int a=0;
  	//thr_id = pthread_create(&p_thread, NULL, turtlebotmap, (void*)&a);

	//signal(SIGINT, handler);
	cout << "Go to main.cpp while roop" << endl;
  	while(1)
  	{
    	ros::spinOnce();
		//ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

    	turtlebotMove();
		cout << "Complete turtlebotMove one time!!" << endl;
		if(isKill)
			break;

		rate.sleep();
  	}
  	isKill = true;
  	//pthread_join(p_thread, 0);

  	return 0;
}


int turtlebotMove()
{
  	geometry_msgs::Twist geo_msg;

  	if(depthCheck_depth())  //장애물 있을 때.
  	{
    	nav_msgs::Odometry msg;
    	double goalangle;

    	goalangle = dothisAngle();

		geo_msg.linear.x = 0;
		geo_msg.angular.z = 0;
  		g_pubGeo.publish(geo_msg);

		//calcDistance();

		// 목표 각도와 유사하게 맞출 때까지 조정한다
    	while(1)
    	{
      		mutex[0].lock();
         	msg = g_odom;
      		mutex[0].unlock();

      		double angle = atan2((2*msg.pose.pose.orientation.w*msg.pose.pose.orientation.z),(1-2*msg.pose.pose.orientation.z*msg.pose.pose.orientation.z));

      		//std::cout << angle << ", " << goalangle << std::endl;

      		if(goalangle - 0.1 <= angle && angle <= goalangle + 0.1)
      		{
        		break;
      		}

      		else if(angle > goalangle)
      		{
  		  		geo_msg.linear.x = 0;
  		  		geo_msg.angular.z = -0.5;
        		g_pubGeo.publish(geo_msg);
      		}
      		else if(angle<goalangle)
      		{
        		geo_msg.linear.x = 0;
  		  		geo_msg.angular.z = 0.5;
        		g_pubGeo.publish(geo_msg);
      		}
    	}
		cout << "Finish Rotation!!" << endl;
  	}
  	else  //장애물 없을 때.
  	{
		cout << "turtlebotMove's else" << endl;
    	geo_msg.linear.x = 0.2;
    	geo_msg.angular.z = 0;
    	g_pubGeo.publish(geo_msg);
  	}

  	return 0;
}

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

/*
double distancelimit() //거리입력
{
    double return_value = 0;

    std::cout << "Input exit distance (>1 Meter) : ";
    std::cin >> return_value;

    if(return_value < 1)
    {
      std::string message = "input_error";
      error_Handle(message);
    }

    return return_value;
}
*/
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

//using namespace std;


int **create2DArray(int height, int width)
{
	int **temp;
	temp = new int*[height];
	for(int i = 0; i < height; i++) {
		temp[i] = new int[width];
	}
	return temp;
}

void delete2DArray(int **ary, int height, int width)
{
	for(int i = 0; i < height; i++) {
		delete[] ary[i];
	}
	delete[] ary;
}

void SelectDirection(double &returnX, double &returnY, double &returnRadian)
{
	cout << "START!!!!!!!!!!!!!!!!" << endl;
	nav_msgs::OccupancyGrid grid;
	nav_msgs::Odometry odom;

	mutex[0].lock();{
	odom = g_odom;
	}mutex[0].unlock();

	mutex[1].lock();{
	grid = g_map;
	}mutex[1].unlock();


  	int width = grid.info.width;
  	int height = grid.info.height;
  	int map_origin_x = grid.info.origin.position.x;
  	int map_origin_y = grid.info.origin.position.y;
  	double map_origin_angle = atan2(2 * grid.info.origin.orientation.w * grid.info.origin.orientation.z, 1 - 2 * pow(grid.info.origin.orientation.z, 2));
	// 방향을 선택하는데 사용하는 변수
	int minValue = 99999999;
	int minX, minY;

	/*
	* 현재 위치 X, Y를 원점을 기준으로 radian 만큼 회전했을 때의 좌표 (The position when the current position X, Y is rotated by radian with respect to the origin)
  	* mX = X * cos(radian) - Y * sin(radian)
	* mY = X * sin(radian) + Y * cos(radian)
  	*/
	// 지도상에서 Turtlebot의 위치
  	int turtlebot_x = (odom.pose.pose.position.x - map_origin_x) / grid.info.resolution * cos(map_origin_angle) - (odom.pose.pose.position.y - map_origin_y) / grid.info.resolution * sin(map_origin_angle);
  	int turtlebot_y = (odom.pose.pose.position.x - map_origin_x) / grid.info.resolution * sin(map_origin_angle) + (odom.pose.pose.position.y - map_origin_y) / grid.info.resolution * cos(map_origin_angle);
	//cout << "odom.position.x: " << setw(2) << setfill('0') << odom.pose.pose.position.x << ", odom.position.y: " << setw(2) << setfill('0') << odom.pose.pose.position.y << endl;

	double TurtlebotSize = 1;
	int turtlebot_size_in_map = (int)(TurtlebotSize / grid.info.resolution);

  	// 일차원 배열인 Gmapping 맵을 이차원 배열로 변환한다
	int **tempMap = create2DArray(height, width);
  	for(int y = 0; y < height; y++) {
    	for(int x = 0; x < width; x++) {
      		int index = width * y + x;
			tempMap[y][x] = grid.data[index];
    	}
  	}
	// 8방향을 뒤져서 가장 멀리까지 갈 수 있을 것으로 예상되는 방향으로 가자
	directionValue dv[8];
	for(int i = 0; i < 8; i++)
		dv[i].value = 0;
	// up
	for(int i = 0; turtlebot_y - i >= 0; i++) {
		if(tempMap[turtlebot_y - i][turtlebot_x] > 1) {
			dv[0].dY = turtlebot_y - i;
			dv[0].dX = turtlebot_x;
			break;
		}
		else if (tempMap[turtlebot_y - i][turtlebot_x] < 0)
			dv[0].value++;
		dv[0].value++;
	}
	// up-right
	for(int i = 0; turtlebot_y - i >= 0 && turtlebot_x + i < width; i++) {
		if(tempMap[turtlebot_y - i][turtlebot_x + i] > 1) {
			dv[1].dY = turtlebot_y - i;
			dv[1].dX = turtlebot_x + i;
			break;
		}
		if(tempMap[turtlebot_y - i][turtlebot_x + i] < 0)
			dv[1].value++;
		dv[1].value++;
	}
	// right
	for(int i = 0; turtlebot_x + i < width; i++) {
		if(tempMap[turtlebot_y][turtlebot_x + i] > 1) {
			dv[2].dY = turtlebot_y;
			dv[2].dX = turtlebot_x + i;
			break;
		}
		else if(tempMap[turtlebot_y][turtlebot_x + i] < 0)
			dv[2].value++;
		dv[2].value++;
	}
	// right-down
	for(int i = 0; turtlebot_x + i < width && turtlebot_y + i < height; i++) {
		if(tempMap[turtlebot_y + i][turtlebot_x + i] > 1) {
			dv[3].dY = turtlebot_y + i;
			dv[3].dX = turtlebot_x + i;
			break;
		}
		else if(tempMap[turtlebot_y + i][turtlebot_x + i] < 0)
			dv[3].value++;
		dv[3].value++;
	}
	// down
	for(int i = 0; turtlebot_y + i < height; i++) {
		if(tempMap[turtlebot_y + i][turtlebot_x] > 1) {
			dv[4].dY = turtlebot_y + i;
			dv[4].dX = turtlebot_x;
			break;
		}
		else if(tempMap[turtlebot_y + i][turtlebot_x] < 0)
			dv[4].value++;
		dv[4].value++;
	}
	// down-left
	for(int i = 0; turtlebot_y + i < height && turtlebot_x - i >= 0; i++) {
		if(tempMap[turtlebot_y + i][turtlebot_x - i] > 1) {
			dv[5].dY = turtlebot_y + i;
			dv[5].dX = turtlebot_x - i;
			break;
		}
		else if(tempMap[turtlebot_y + i][turtlebot_x - i] < 0)
			dv[5].value++;
		dv[5].value++;
	}
	// left
	for(int i = 0; turtlebot_x - i >= 0; i++) {
		if(tempMap[turtlebot_y][turtlebot_x - i] > 1){
			dv[6].dY = turtlebot_y;
			dv[6].dX = turtlebot_x - i;
			break;
		}
		else if(tempMap[turtlebot_y][turtlebot_x - i] < 0)
			dv[6].value++;
		dv[6].value++;
	}
	// left-up
	for(int i = 0; turtlebot_x - i >= 0 && turtlebot_y - i >= 0; i++) {
		if(tempMap[turtlebot_y - i][turtlebot_x - i] > 1) {
			dv[7].dY = turtlebot_y - i;
			dv[7].dX = turtlebot_x - i;
			break;
		}
		else if(tempMap[turtlebot_y - i][turtlebot_x - i] < 0)
			dv[7].value++;
		dv[7].value++;
	}

	// 각 방향 중 가장 가치가 높은(검은색이 많고 장애물이 멀리 있는) 방향을 고른다
	int maxIndex = 0;
	for(int i = 0; i < 8; i++) {
		if(dv[maxIndex].value < dv[i].value)
			maxIndex = i;
	}
	minX = dv[maxIndex].dX;
	minY = dv[maxIndex].dY;
	/*
	* 현재 위치 X, Y를 원점을 기준으로 radian 만큼 회전했을 때의 좌표 (The position when the current position X, Y is rotated by radian with respect to the origin)
  	* mX = X * cos(radian) - Y * sin(radian)
	* mY = X * sin(radian) + Y * cos(radian)
  	*/
	// Odom 좌표계 상에서 목적지의 위치
  	double odom_destination_x = (minX + map_origin_x) * grid.info.resolution * cos(map_origin_angle) - (minY + map_origin_y) * grid.info.resolution * sin(map_origin_angle);
  	double odom_destination_y = (minX + map_origin_x) * grid.info.resolution * sin(map_origin_angle) + (minY + map_origin_y) * grid.info.resolution * cos(map_origin_angle);
	double odom_destination_radian = atan2(odom_destination_y - odom.pose.pose.position.y, odom_destination_x - odom.pose.pose.position.x);
	//double odom_destination_radian = atan2(minY - turtlebot_y, minX - turtlebot_x);

	// return by referance
	returnX = odom_destination_x;
	returnY = odom_destination_y;
	returnRadian = odom_destination_radian;

	cout << "Select Direction: " << odom_destination_radian << endl;
}

/*
// Version 1
void SelectDirection(double &returnX, double &returnY, double &returnRadian)
{
	cout << "START!!!!!!!!!!!!!!!!" << endl;
	nav_msgs::OccupancyGrid grid;
	nav_msgs::Odometry odom;

	mutex[0].lock(); {
	odom = g_odom;
	} mutex[0].unlock();
	mutex[2].lock(); {
	grid = g_map;
	} mutex[2].unlock();


  	int width = grid.info.width;
  	int height = grid.info.height;
  	int map_origin_x = grid.info.origin.position.x;
  	int map_origin_y = grid.info.origin.position.y;
  	double map_origin_angle = atan2(2 * grid.info.origin.orientation.w * grid.info.origin.orientation.z, 1 - 2 * pow(grid.info.origin.orientation.z, 2));
	// 방향을 선택하는데 사용하는 변수
	int minValue = 99999999;
	int minX, minY;


	//* 현재 위치 X, Y를 원점을 기준으로 radian 만큼 회전했을 때의 좌표 (The position when the current position X, Y is rotated by radian with respect to the origin)
  	//* mX = X * cos(radian) - Y * sin(radian)
	//* mY = X * sin(radian) + Y * cos(radian)

	// 지도상에서 Turtlebot의 위치
  	int turtlebot_x = (odom.pose.pose.position.x - map_origin_x) / grid.info.resolution * cos(map_origin_angle) - (odom.pose.pose.position.y - map_origin_y) / grid.info.resolution * sin(map_origin_angle);
  	int turtlebot_y = (odom.pose.pose.position.x - map_origin_x) / grid.info.resolution * sin(map_origin_angle) + (odom.pose.pose.position.y - map_origin_y) / grid.info.resolution * cos(map_origin_angle);
	//cout << "odom.position.x: " << setw(2) << setfill('0') << odom.pose.pose.position.x << ", odom.position.y: " << setw(2) << setfill('0') << odom.pose.pose.position.y << endl;

	double TurtlebotSize = 1;
	int turtlebot_size_in_map = (int)(TurtlebotSize / grid.info.resolution);

  	// 일차원 배열인 Gmapping 맵을 이차원 배열로 변환한다
	int **tempMap = create2DArray(height, width);
  	for(int y = 0; y < height; y++) {
    	for(int x = 0; x < width; x++) {
      		int index = width * y + x;
			tempMap[y][x] = grid.data[index];
    	}
  	}
	// 2차원 배열로 변경한 Map에서 n pixel씩 sliding하며 값을 계산한다
	int unitPixel = 3;
	int start_x = turtlebot_x - 1 * turtlebot_size_in_map;
	int start_y = turtlebot_y - 1 * turtlebot_size_in_map;
	if(start_x < 0)
		start_x = 0;
	if(start_y < 0)
		start_y = 0;
	for(start_y; start_y + turtlebot_size_in_map < turtlebot_y + 1 * turtlebot_size_in_map && start_y + turtlebot_size_in_map < height; start_y += unitPixel) {
    	for(start_x = 0; start_x + turtlebot_size_in_map < turtlebot_x + 1 * turtlebot_size_in_map && start_x + turtlebot_size_in_map < width; start_x += unitPixel) {
			// 값을 계산하기 위해 한 변이 turtlebot_size_in_map인 정사각형을 순회한다
			int tempValue = 0;
			for(int y = start_y; y < start_y + turtlebot_size_in_map && y < height; y++) {
				for(int x = start_x; x < start_x + turtlebot_size_in_map && y < width; x++) {
					// 지도에서 "> 1"은 장애물
					// 지도에서 "= 0"은 탐색한 공간
					// 지도에서 "< 0"은 아직 탐색하지 않은 공간
					if(tempMap[y][x] > 1)
						tempValue += 1000;
					else if(tempMap[y][x] == 0)
						tempValue += 0;
					else
						tempValue += 100;
				}
			}
			// 구한 값이 최소값이면 minX, minY를 업데이트한다.
			if(tempValue < minValue) {
				minValue = tempValue;
				// 목표로 할 X, Y값은 현재 window(sector)의 중앙 위치로 한다
				// 범위를 벗어나면 보정해준다
				cout << "start_x: " << start_x << ", start_y: " << start_y << endl;
				cout << "Value: " << tempValue << endl;
				minX = (start_x + turtlebot_size_in_map) / 2;
				if(minX >= width)
					minX = width - 1;
				minY = (start_y + turtlebot_size_in_map) / 2;
				if(minY >= height)
					minY = height - 1;
			}
    	}
  	}
	delete2DArray(tempMap, height, width);

	//* 현재 위치 X, Y를 원점을 기준으로 radian 만큼 회전했을 때의 좌표 (The position when the current position X, Y is rotated by radian with respect to the origin)
  	//* mX = X * cos(radian) - Y * sin(radian)
	//* mY = X * sin(radian) + Y * cos(radian)

	// Odom 좌표계 상에서 목적지의 위치
  	double odom_destination_x = (minX + map_origin_x) * grid.info.resolution * cos(map_origin_angle) - (minY + map_origin_y) * grid.info.resolution * sin(map_origin_angle);
  	double odom_destination_y = (minX + map_origin_x) * grid.info.resolution * sin(map_origin_angle) + (minY + map_origin_y) * grid.info.resolution * cos(map_origin_angle);
	double odom_destination_radian = atan2(odom_destination_y - odom.pose.pose.position.y, odom_destination_x - odom.pose.pose.position.x);
	//double odom_destination_radian = atan2(minY - turtlebot_y, minX - turtlebot_x);

	// return by referance
	returnX = odom_destination_x;
	returnY = odom_destination_y;
	returnRadian = odom_destination_radian;

	cout << "Select Direction: " << odom_destination_radian << endl;
}
*/
