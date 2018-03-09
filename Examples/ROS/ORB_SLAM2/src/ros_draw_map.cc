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

#include <math.h>
#include <vector>
#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

double orb_scale = 1.0;
tf::Transform map_transform;

void publish_points(visualization_msgs::Marker& points, float x, float y, float z)
{
	tf::Vector3 pose(x*orb_scale, z*orb_scale, y*orb_scale);
	tf::Vector3 transformed_pose = map_transform * pose;

	geometry_msgs::Point p;
	p.x = (float) transformed_pose.getX();
	p.y = (float) transformed_pose.getY();
	p.z = (float) transformed_pose.getZ();
	points.points.push_back(p);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MAP");
	ros::start();
	ros::NodeHandle nh;
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker> ("map_marker", 10);
	ros::Rate r(30);

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
	points.scale.x = 0.02;
	points.scale.y = 0.02;
	points.color.b = 1.0f;
	points.color.a = 1.0;

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
		return 1;
	}

	for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
	{
		if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
			continue;
		cv::Mat pos = vpMPs[i]->GetWorldPos();
		publish_points(points, pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
	}

	while (ros::ok())
	{
		points.header.stamp = ros::Time::now();
		marker_pub.publish(points);
		r.sleep();
	}
	SLAM.Shutdown();
}
