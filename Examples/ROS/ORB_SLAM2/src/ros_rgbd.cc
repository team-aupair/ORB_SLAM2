/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
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

#include "pepper_obj_msgs/objs_array.h"
#include "pepper_obj_msgs/objs.h"

volatile sig_atomic_t flag = 0;
double orb_scale = 1.0;
vector<double> orb_trans;
vector<double> orb_quat;
tf::Transform map_transform;

ORB_SLAM2::Map* mpMap;

const int map_update_rate = 5;
int update_counter = map_update_rate;

typedef boost::shared_ptr< ::pepper_obj_msgs::objs_array const> ObjConstPtr;

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const ObjConstPtr& msgC, ros::NodeHandle& nh, ros::Publisher& marker_pub);

    ORB_SLAM2::System* mpSLAM;
};

void keyboard_inturrupt(int sig){
    flag = 1;
}

void publish_points(visualization_msgs::Marker& points, float x, float y, float z, unsigned char s)
{
	tf::Vector3 pose(x*orb_scale, z*orb_scale, -y*orb_scale);
	tf::Vector3 transformed_pose = map_transform * pose;

	geometry_msgs::Point p;
	p.x = (float) transformed_pose.getX();
	p.y = (float) transformed_pose.getY();
	p.z = (float) transformed_pose.getZ();
	points.points.push_back(p);

  std_msgs::ColorRGBA c;
  if(s == 1){
    c.r = 1.0;
    c.g = 0.0;
    c.b = 0.0;
  }
  else if(s == 2){
    c.r = 0.0;
    c.g = 1.0;
    c.b = 0.0;
  }
  else if(s == 3){
    c.r = 0.0;
    c.g = 0.0;
    c.b = 1.0;
  }
  else if(s == 4){
    c.r = 1.0;
    c.g = 1.0;
    c.b = 0.0;
  }
  else if(s == 5){
    c.r = 1.0;
    c.g = 0.0;
    c.b = 1.0;
  }
  else{
    c.r = 0.0;
    c.g = 1.0;
    c.b = 1.0;
  }
  c.a = 1.0;
  points.colors.push_back(c);
}

void get_map_points(visualization_msgs::Marker& points)
{
	points.header.frame_id = "/map";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.025;
	points.scale.y = 0.025;
	points.color.b = 1.0f;
	points.color.a = 1.0;

	tf::Vector3 map_translation(orb_trans[0], orb_trans[1], orb_trans[2]);
	tf::Quaternion map_quaternion(orb_quat[0], orb_quat[1], orb_quat[2], orb_quat[3]);
	map_transform = tf::Transform(map_quaternion, map_translation);

	const vector<ORB_SLAM2::MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
	const vector<ORB_SLAM2::MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();


	set<ORB_SLAM2::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

	if(vpMPs.empty()){
		return;
	}

	for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
	{
		//if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
		if(vpMPs[i]->isBad())
			continue;
		cv::Mat pos = vpMPs[i]->GetWorldPos();
    cv::Mat dcptr = vpMPs[i]->GetDescriptor();
		publish_points(points, pos.at<float>(0),pos.at<float>(1),pos.at<float>(2), dcptr.at<unsigned char>(32));
	}
}

cv::Mat getObjImg(cv::Mat& img, vector<pepper_obj_msgs::objs>& obj_list)
{
  for(int i=0; i<obj_list.size(); i++)
  {
    pepper_obj_msgs::objs obj = obj_list[i];
    int color = 0;

    if (obj.class_string == "person") color = 1;
    else if (obj.class_string == "bottle" || obj.class_string == "coke" || obj.class_string == "green tea" || obj.class_string == "aquarius") color = 2;
    else if (obj.class_string == "chair" || obj.class_string == "table" || obj.class_string == "diningtable") color = 3;
    else if (obj.class_string == "tvmonitor" || obj.class_string == "keyboard" || obj.class_string == "clock") color = 4;
    else if (obj.class_string == "sofa" || obj.class_string == "refrigerator") color = 5;

    cv::Rect rc(obj.y,obj.x,obj.w,obj.h);
    cv::rectangle(img, rc, color, CV_FILLED);	// filled rectangle
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true, true);
	mpMap = SLAM.getMap();

	ros::NodeHandle nh;

	cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
	orb_scale = (double) fsSettings["Map.scale"];
	nh.setParam("orb_scale", orb_scale);

	cv::FileNode map_t_ = fsSettings["Map.translation"];
	cv::FileNode map_q_ = fsSettings["Map.quaternion"];

	for(cv::FileNodeIterator it = map_t_.begin(); it!=map_t_.end(); ++it)
	    orb_trans.push_back((double)*it);

	for(cv::FileNodeIterator it = map_q_.begin(); it!=map_q_.end(); ++it)
	    orb_quat.push_back((double)*it);

	nh.setParam("orb_translation", orb_trans);
	nh.setParam("orb_quaternion", orb_quat);

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker> ("map_marker", 10);

    ImageGrabber igb(&SLAM);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/pepper_robot/camera/front/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/pepper_robot/camera/depth/image_raw", 1);
    message_filters::Subscriber<pepper_obj_msgs::objs_array> caption_sub(nh, "/objects", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, pepper_obj_msgs::objs_array> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub,caption_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2,_3,nh,marker_pub));

    signal(SIGINT, keyboard_inturrupt);

    ros::Rate r(10);

    while (flag == 0){
        ros::spinOnce();
	r.sleep();
    }
    cout << "OUT!" << endl;
    // Stop all threads
    SLAM.Shutdown();
    cout << "MAP SAVED!" << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const ObjConstPtr& msgC, ros::NodeHandle& nh, ros::Publisher& marker_pub)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    vector<pepper_obj_msgs::objs> obj_list =  msgC->objects;
    cv::Mat objmap = cv::Mat::zeros(cv_ptrRGB->image.rows, cv_ptrRGB->image.cols, CV_8UC1);
    getObjImg(objmap, obj_list);
    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image, objmap, cv_ptrRGB->header.stamp.toSec());

    if (pose.empty())
        return;

	nh.getParam("orb_scale", orb_scale);
	nh.getParam("orb_translation", orb_trans);
	nh.getParam("orb_quaternion", orb_quat);

	tf::Matrix3x3 rh_cameraPose(pose.at<float>(0, 0), pose.at<float>(0, 2), pose.at<float>(0, 1),
		-pose.at<float>(1, 0), -pose.at<float>(1, 2), -pose.at<float>(1, 1),
		pose.at<float>(2, 0), pose.at<float>(2, 2), pose.at<float>(2, 1));

	tf::Vector3 rh_cameraTranslation(pose.at<float>(0, 3)*orb_scale, -pose.at<float>(1, 3)*orb_scale, pose.at<float>(2, 3)*orb_scale);


/*	tf::Matrix3x3 rh_cameraPose_T = rh_cameraPose.inverse();
	tf::Vector3 cameraPosition(
	  - rh_cameraPose_T.getRow(0).dot(rh_cameraTranslation) * orb_scale,
	  - rh_cameraPose_T.getRow(1).dot(rh_cameraTranslation) * orb_scale,
	  - rh_cameraPose_T.getRow(2).dot(rh_cameraTranslation) * orb_scale
	);

	cv::Mat R_inv = (cv::Mat_<double>(3, 3) <<
	   rh_cameraPose_T.getRow(0)[0], rh_cameraPose_T.getRow(0)[1], rh_cameraPose_T.getRow(0)[2],
	   rh_cameraPose_T.getRow(1)[0], rh_cameraPose_T.getRow(1)[1], rh_cameraPose_T.getRow(1)[2],
	   rh_cameraPose_T.getRow(2)[0], rh_cameraPose_T.getRow(2)[1], rh_cameraPose_T.getRow(2)[2]);

	// pan(yaw) & tilt(pitch)
	double unit_z[] = {0,0,1};
	cv::Mat Zc(3, 1, CV_64FC1, unit_z);
	cv::Mat Zw = R_inv*Zc;		// world coordinate of optical axis

	double* zw = (double *)Zw.data;
	double pan = atan2(zw[1], zw[0]) - CV_PI/2;
	double tilt = atan2(zw[2], sqrt(zw[0]*zw[0]+zw[1]*zw[1]));

	// roll
	double unit_x[] = {1,0,0};
	cv::Mat Xc(3, 1, CV_64FC1, unit_x);
	cv::Mat Xw = R_inv*Xc;		// world coordinate of camera X axis
	double* xw = (double *)Xw.data;
	double xpan[] = {cos(pan), sin(pan), 0};

	double roll = acos(xw[0]*xpan[0] + xw[1]*xpan[1] + xw[2]*xpan[2]); // inner product
	if(xw[2]<0) roll = -roll;

	tf::Matrix3x3 rotation(tf::Quaternion(pan, tilt, roll));
        /*tf::Matrix3x3 rotation(
            -rot_.getRow(0)[0], rot_.getRow(0)[1], rot_.getRow(0)[2],
            -rot_.getRow(1)[0], rot_.getRow(1)[1], rot_.getRow(1)[2],
            rot_.getRow(2)[0], -rot_.getRow(2)[1], -rot_.getRow(2)[2]);
        tf::Matrix3x3 rotation270degZx(
            0, 0, 1,
            -1, 0, 0,
            0, -1, 0);
        rotation = rotation270degZx.inverse() * rotation;*/

	static tf::TransformBroadcaster br;
	tf::Vector3 map_translation(orb_trans[0], orb_trans[1], orb_trans[2]);
	tf::Quaternion map_quaternion(orb_quat[0], orb_quat[1], orb_quat[2], orb_quat[3]);
	tf::Transform transformMap = tf::Transform(map_quaternion, map_translation);
	br.sendTransform(tf::StampedTransform(transformMap, ros::Time::now(), "map", "orb_map"));

	tf::TransformListener listener_link;
	tf::StampedTransform transform_link;
	try {
		listener_link.waitForTransform("/CameraTop_optical_frame", "/base_link", ros::Time(0), ros::Duration(10.0));
		listener_link.lookupTransform("/CameraTop_optical_frame", "/base_link", ros::Time(0), transform_link);
		transform_link.frame_id_ = "orb_pose";
		transform_link.child_frame_id_ = "orb_base_link";
		br.sendTransform(transform_link);
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}

	tf::Transform transformCamera = tf::Transform(rh_cameraPose, rh_cameraTranslation);
	br.sendTransform(tf::StampedTransform(transformCamera.inverse(), ros::Time::now(), "orb_map", "orb_pose"));

	//publish map
	/*tf::TransformListener listener_link;
	tf::TransformListener listener_pose;
	tf::StampedTransform transform_link;
	tf::StampedTransform transform_pose;
	try {
		listener_link.lookupTransform("/CameraTop_optical_frame", "/pose", ros::Time(0), transform_link);
		listener_pose.lookupTransform("/camera_pose", "/pose", ros::Time(0), transform_pose);
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}

	cout << "CAMERA_ODOM : (" << transform_link.getOrigin().x() << ", " << transform_link.getOrigin().y() << ", " << transform_link.getOrigin().z() << ")" << endl;
	cout << "CAMERA_POSE : (" << transform_pose.getOrigin().x() << ", " << transform_pose.getOrigin().y() << ", " << transform_pose.getOrigin().z() << ")" << endl;*/

	if(update_counter++ == map_update_rate) {
		visualization_msgs::Marker points;
		get_map_points(points);
		points.header.stamp = ros::Time::now();
		marker_pub.publish(points);
		update_counter = 1;
	}
}
