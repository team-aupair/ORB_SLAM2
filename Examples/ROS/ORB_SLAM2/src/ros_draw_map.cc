#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include "geometry_msgs/TransformStamped.h"
#include "../../../include/System.h"
#include "../../../include/Map.h"
#include "../../../include/MapPoint.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

#include <math.h>
#include <vector>
#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

double orb_scale = 1.0;
tf::Transform map_transform;
float l = 0.0;
float r = 0.0;
float u = 0.0;
float b = 0.0;
bool flag = true;

int threshold = 5;
float size = 0.01;

void publish_points(visualization_msgs::Marker& points, float x, float y, float z)
{
	tf::Vector3 pose(x*orb_scale, z*orb_scale, y*orb_scale);
	tf::Vector3 transformed_pose = map_transform * pose;

	geometry_msgs::Point p;
	p.x = (float) transformed_pose.getX();
	p.y = (float) transformed_pose.getY();
	p.z = (float) transformed_pose.getZ();
	points.points.push_back(p);

	if(!flag) {
		l = (l < p.x ? l : p.x);
		r = (r < p.x ? p.x : r);
		u = (u < p.y ? p.y : u);
		b = (b < p.y ? b : p.y);
	}
	else {
		l = p.x;
		r = p.x;
		u = p.y;
		b = p.y;
		flag = false;
	}
}

void make_map(std::vector<std::vector<int>>& map_matrix, float x, float y, float z)
{
	tf::Vector3 pose(x*orb_scale, z*orb_scale, y*orb_scale);
	tf::Vector3 transformed_pose = map_transform * pose;

	geometry_msgs::Point p;
	int map_x = (int) ((transformed_pose.getX() - l) / size);
	int map_y = (int) ((transformed_pose.getY() - b) / size);
//	std::cout << map_x << ", " << map_y << std::endl;

	for(int i=-2; i<3; i++){
		if (map_y+i < 0 || map_y+i >= (int)((u-b) / size )+1 )
			continue;
		for(int j=-2; j<3; j++){
			if (map_x+j < 0 || map_x+j >= (int)((r-l) / size )+1 )
				continue;
			map_matrix[map_y+i][map_x+j] += 1;
		}
	}
}

void publish_map(visualization_msgs::Marker& map, nav_msgs::OccupancyGrid& grid_map, std::vector<std::vector<int>>& map_matrix, int rows, int cols)
{
	int index = 0;
	grid_map.header.frame_id = "/map";
	grid_map.info.resolution = size;
	grid_map.info.width = cols;
	grid_map.info.height = rows;
	grid_map.info.origin.position.x = l;
	grid_map.info.origin.position.y = b;
	grid_map.info.origin.position.z = 0.0;
	grid_map.info.origin.orientation.x = 0.0;
	grid_map.info.origin.orientation.y = 0.0;
	grid_map.info.origin.orientation.z = 0.0;
	grid_map.info.origin.orientation.w = 1.0;
	grid_map.data.resize(cols*rows);

	for(int i=0; i<rows; i++){
		for(int j=0; j<cols; j++){
			geometry_msgs::Point p;
			std_msgs::ColorRGBA c;
			p.x = l + j * size;
			p.y = b + i * size;
			p.z = 0.0;
			if(map_matrix[i][j] >= threshold){
				c.r = c.g = c.b = 0.0;
				grid_map.data[index] = 100;
			}
			else{
				c.r = c.g = c.b = 1.0;
				grid_map.data[index] = 0;
			}
			c.a = 1.0;
			index++;

			map.points.push_back(p);
			map.colors.push_back(c);
		}
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "MAP");
	ros::start();
	ros::NodeHandle nh;
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker> ("map_marker", 10);
	ros::Publisher maker_pub = nh.advertise<visualization_msgs::Marker> ("map_maker", 10);
	ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid> ("/map", 10);
	ros::Rate rate(1000);
	nav_msgs::OccupancyGrid grid_map;

	if(argc != 3)
	{
		cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
		ros::shutdown();
		return 1;
	}    

	visualization_msgs::Marker points;
	points.header.frame_id = "/map";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.025;
	points.scale.y = 0.025;
	points.color.b = 1.0f;
	points.color.a = 1.0;

	visualization_msgs::Marker map;
	map.header.frame_id = "/map";
	map.action = visualization_msgs::Marker::ADD;
	map.pose.orientation.w = 1.0;
	map.id = 1;
	map.type = visualization_msgs::Marker::CUBE_LIST;
	map.scale.x = 0.01;
	map.scale.y = 0.01;
	map.scale.z = 0.001;
	map.color.r = 0.0f;
	map.color.g = 0.0f;
	map.color.b = 0.0f;
	map.color.a = 1.0;

	/*visualization_msgs::Marker map_back;
	map_back.header.frame_id = "/map";
	map_back.action = visualization_msgs::Marker::ADD;
	map_back.pose.orientation.w = 1.0;
	map_back.id = 2;
	map_back.type = visualization_msgs::Marker::CUBE;
	map_back.color.r = 1.0f;
	map_back.color.g = 1.0f;
	map_back.color.b = 1.0f;
	map_back.color.a = 1.0;*/

	vector<double> orb_trans;
	vector<double> orb_quat;
	cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
	orb_scale = (double) fsSettings["Map.scale"];

	cv::FileNode map_t_ = fsSettings["Map.translation"];
	cv::FileNode map_q_ = fsSettings["Map.quaternion"];

	for(cv::FileNodeIterator it = map_t_.begin(); it!=map_t_.end(); ++it)
	    orb_trans.push_back((double)*it);

	for(cv::FileNodeIterator it = map_q_.begin(); it!=map_q_.end(); ++it)
	    orb_quat.push_back((double)*it);

	tf::Vector3 map_translation(orb_trans[0], orb_trans[1], orb_trans[2]);
	tf::Quaternion map_quaternion(orb_quat[0], orb_quat[1], orb_quat[2], orb_quat[3]);
	map_transform = tf::Transform(map_quaternion, map_translation);

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true, true);
	ORB_SLAM2::Map* mpMap = SLAM.getMap();
	const vector<ORB_SLAM2::MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
	const vector<ORB_SLAM2::MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();


	set<ORB_SLAM2::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

	if(vpMPs.empty()){
		SLAM.Shutdown();
		ros::shutdown();
		return 1;
	}

	for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
	{
		if(vpMPs[i]->isBad())
			continue;
		cv::Mat pos = vpMPs[i]->GetWorldPos();
		publish_points(points, pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
	}
	u-=23;
	b+=25;
	r-=10;
	l+=6;

	/*map_back.scale.x = r-l;
	map_back.scale.y = u-b;
	map_back.scale.z = 0.001;
	map_back.pose.position.x = (r+l)/2;
	map_back.pose.position.y = (u+b)/2;
	map_back.pose.position.z = -0.001;*/


	int cols = (int)((r-l) / size)+1;
	int rows = (int)((u-b) / size)+1;
	std::cout << "rows: " << rows << " | cols: " << cols << std::endl;
	std::vector<std::vector<int>> map_matrix(rows, std::vector<int>(cols));
	//map_matrix.resize(rows, std::vector<int>(cols, 0));

	for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
	{
		if(vpMPs[i]->isBad())
			continue;
		cv::Mat pos = vpMPs[i]->GetWorldPos();
		make_map(map_matrix, pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
	}

	publish_map(map, grid_map, map_matrix, rows, cols);

	points.header.stamp = ros::Time::now();
	marker_pub.publish(points);

	std::cout << grid_map.info.origin.position << std::endl;
	while (ros::ok())
	{
		rate.sleep();
		map.header.stamp = ros::Time::now();
		maker_pub.publish(map);
		grid_map.header.stamp = ros::Time::now();
		map_pub.publish(grid_map);
		//maker_pub.publish(map_back);
	}

	SLAM.Shutdown();
	ros::shutdown();
}
