#include "header.h"

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

  	ros::Rate rate(1000);
    srand(time(NULL));
  	double limit_distance = distancelimit();
  	pthread_t p_thread;
  	int thr_id;
  	int status;
  	int a=0;
  	//thr_id = pthread_create(&p_thread, NULL, turtlebotmap, (void*)&a);

	//signal(SIGINT, handler);
	cout << "Go to main.cpp while roop" << endl;
  	while(g_totaldistance <= limit_distance)
  	{
    	//ros::spinOnce();
		//rate.sleep();

		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

    	turtlebotMove();
		cout << "Complete turtlebotMove one time!!" << endl;
		if(isKill)
			break;
  	}
  	isKill = true;
  	//pthread_join(p_thread, 0);

  	return 0;
}
