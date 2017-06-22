#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>


using namespace cv;
using namespace std;


///////////////////////////////////Macro///////////////////////////////////////////////////////////////////////////////////////////
#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))
#define ROWS 800
#define COLS 800



/////////////////////////////////////Global variable/////////////////////////////////////////////////////////////////////////////////////////
// mutex[0] ==> g_odom
// mutex[1] ==> g_scan
// mutex[2] ==> g_grid
static boost::mutex mutex[3];
static nav_msgs::Odometry g_odom;
static sensor_msgs::LaserScan g_scan;
static nav_msgs::OccupancyGrid g_grid;
static Mat display;
static double ENLARGEMENT = 1;




////////////////////////////////////A template method to check 'nan'//////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
inline bool isnan(T value)
{
  return value != value;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int getBigger(int a, int b)
{
	if(b > a)
		return b;
	else
		return a;
}



//////////////////////////////////////callback function////////////////////////////////////////////////////////////////////////////////////////
void odomMsgCallback(const nav_msgs::Odometry &msg)
{
  // receive a '/odom' message with the mutex
  mutex[0].lock(); {
    g_odom = msg;
  } mutex[0].unlock();
}


///////////////////////////////////callback function///////////////////////////////////////////////////////////////////////////////////////////
void scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
  // receive a '/odom' message with the mutex
  mutex[1].lock(); {
    g_scan = msg;
  } mutex[1].unlock();
}

///////////////////////////////////callback function///////////////////////////////////////////////////////////////////////////////////////////
void occupancyGrid_Callback(const nav_msgs::OccupancyGrid grid)
{
  // receive a '/odom' message with the mutex
  mutex[2].lock(); {
    g_grid = grid;
  } mutex[2].unlock();
}

void drawOccupancyGrid(Mat &display)
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
  	int turtlebot_x = (g_odom.pose.pose.position.x - map_origin_x) / g_grid.info.resolution * cos(map_origin_angle) - (g_odom.pose.pose.position.y - map_origin_y) / g_grid.info.resolution * sin(map_origin_angle);
  	int turtlebot_y = (g_odom.pose.pose.position.x - map_origin_x) / g_grid.info.resolution * sin(map_origin_angle) + (g_odom.pose.pose.position.y - map_origin_y) / g_grid.info.resolution * cos(map_origin_angle);
  	//int turtlebot_x = (odom.pose.pose.position.x - map_origin_x) / grid.info.resolution * sin(map_origin_angle) + (odom.pose.pose.position.y - map_origin_y) / grid.info.resolution * cos(map_origin_angle);
  	//int turtlebot_y = (odom.pose.pose.position.x - map_origin_x) / grid.info.resolution * cos(map_origin_angle) - (odom.pose.pose.position.y - map_origin_y) / grid.info.resolution * sin(map_origin_angle);
	//cout << "odom.position.x: " << setw(2) << setfill('0') << odom.pose.pose.position.x << ", odom.position.y: " << setw(2) << setfill('0') << odom.pose.pose.position.y << endl;

  	//Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);

  	//int nSize = (int) trajectory.size();
  	
  	// width, height 중 큰 값을 통해서 확대율을 정한다
  	ENLARGEMENT = ROWS / (double)getBigger(height, width);
  	cout << "height: " << height << ", width: " << width << ", ENLARGEMENT: " << setw(2) << setfill('0') << ENLARGEMENT << endl;

  	for(int y = 0; y < height; y++) {
    	for(int x = 0; x < width; x++) {
      		int index = width * y + x;
      		if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
        		if (grid.data[index] > 0) //display.at<Vec3b>(x * ENLARGEMENT, y * ENLARGEMENT) = Vec3b(255, 255, 255);
					display.at<Vec3b>((int)(y * ENLARGEMENT), (int)(x * ENLARGEMENT)) = Vec3b(255, 255, 255);
        		else if (grid.data[index] == 0) //display.at<Vec3b>(x * ENLARGEMENT, y * ENLARGEMENT) = Vec3b(127, 127, 127);
          			display.at<Vec3b>((int)(y * ENLARGEMENT), (int)(x * ENLARGEMENT)) = Vec3b(127, 127, 127);
      		}
    	}
  	}
  	cout << "HERE????????????????????????????????????????????????????????????????????????" << endl;
  	// Current Turtlebot's Position
  	//circle(display, Point((int)(turtlebot_x * ENLARGEMENT), (int)(turtlebot_y * ENLARGEMENT)), (int)(5 * ENLARGEMENT), CV_RGB(0, 0, 255), 1, CV_AA);
  	cout << "HERE????????????????????????????????????????????????????????????????????????" << endl;
  	// Viewing Direction of Turtlebot
  	double turtlebot_angle = atan2(2 * odom.pose.pose.orientation.w * odom.pose.pose.orientation.z, 1 - 2 * pow(odom.pose.pose.orientation.z, 2));
  	double map_turtlebot_angle = turtlebot_angle + map_origin_angle;
  	/*
  	* 위치 x, y에서 length만큼 radian 각도로 이동한 위치
  	* point x = cos(Rad) * length
  	* point y = sin(Rad) * length
  	*/
  	int line_length = 5;
	line(display,
   		Point((int)(turtlebot_x * ENLARGEMENT), (int)(turtlebot_y * ENLARGEMENT)),
    	Point((int)((turtlebot_x + line_length * cos(map_turtlebot_angle)) * ENLARGEMENT), (int)((turtlebot_y + line_length * sin(map_turtlebot_angle)) * ENLARGEMENT)),
     	CV_RGB(255, 0, 255), 1, CV_AA);
	/*
	x, y 뒤집어져있을 때
  	line(display,
   		Point(turtlebot_x * ENLARGEMENT, turtlebot_y * ENLARGEMENT),
    	Point((turtlebot_x + line_length * sin(map_turtlebot_angle)) * ENLARGEMENT, (turtlebot_y + line_length * cos(map_turtlebot_angle)) * ENLARGEMENT),
     	CV_RGB(255, 0, 255), 1, CV_AA);
		*/
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlebot_map");
  	ros::NodeHandle n;

  	ros::Subscriber sub = n.subscribe("map", 131072, occupancyGrid_Callback);
  	ros::Subscriber subOdom = n.subscribe("/odom", 100, &odomMsgCallback);

	//initGrid(display, 1600);
	display = Mat::zeros(800, 800, CV_8UC3);
  	//display = cv::Mat(1, 1, CV_8UC1, cv::Scalar(0)); // init :: gray image

  	while(ros::ok()){
    	ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    	display.setTo(Scalar(0, 0, 0));
    	drawOccupancyGrid(display);
    	cv::imshow("Map", display);
    	cv::waitKey(30);
  	}

  	return 0;
}
