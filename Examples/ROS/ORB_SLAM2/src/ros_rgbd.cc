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
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <math.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

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

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
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

    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    if (pose.empty())
        return;

	/* prev code */
	// transform into right handed camera frame
	tf::Matrix3x3 rh_cameraPose(-pose.at<float>(0, 0), pose.at<float>(0, 1), pose.at<float>(0, 2),
		-pose.at<float>(1, 0), pose.at<float>(1, 1), pose.at<float>(1, 2),
		pose.at<float>(2, 0), -pose.at<float>(2, 1), -pose.at<float>(2, 2));

	tf::Vector3 rh_cameraTranslation(pose.at<float>(0, 3), pose.at<float>(1, 3), -pose.at<float>(2, 3));


	tf::Matrix3x3 rh_cameraPose_T = rh_cameraPose.inverse();
	tf::Vector3 pointing = rh_cameraPose_T.getColumn(2);
	tf::Vector3 cameraPosition(
	  - rh_cameraPose_T.getRow(0).dot(rh_cameraTranslation),
	  - rh_cameraPose_T.getRow(1).dot(rh_cameraTranslation),
	  - rh_cameraPose_T.getRow(2).dot(rh_cameraTranslation)
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

	tf::Quaternion quaternion(pan, tilt, roll);

	//rotate 270deg about z and 270deg about x
	tf::Matrix3x3 rotation270degZX(0, 0, 1,
									-1, 0, 0,
									0, -1, 0);

	//publish right handed, x forward, y right, z down (NED)
	static tf::TransformBroadcaster br;
	//tf::Transform transformCoordSystem = tf::Transform(rotation270degZX, tf::Vector3(0.0, 0.0, 0.0));
	//br.sendTransform(tf::StampedTransform(transformCoordSystem, ros::Time::now(), "asdf", "orb_map"));

	tf::Transform transformCamera = tf::Transform(quaternion, cameraPosition);
	br.sendTransform(tf::StampedTransform(transformCamera, ros::Time::now(), "orb_map", "orb_pose"));

/*
	
    // global left handed coordinate system
    static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
                                                               -1, 1,-1, 1,
                                                               -1,-1, 1, 1,
                                                                1, 1, 1, 1);

    //prev_pose * T = pose
    cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
    world_lh = world_lh * translation;
    pose_prev = pose.clone();


    // transform into global right handed coordinate system, publish in ROS
    tf::Matrix3x3 cameraRotation_rh(  - world_lh.at<float>(0,0),   world_lh.at<float>(0,1),   world_lh.at<float>(0,2),
                                  - world_lh.at<float>(1,0),   world_lh.at<float>(1,1),   world_lh.at<float>(1,2),
                                    world_lh.at<float>(2,0), - world_lh.at<float>(2,1), - world_lh.at<float>(2,2));

    tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

    //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                            0, 0, 1,
                                            1, 0, 0);

    //static tf::TransformBroadcaster br;

    tf::Matrix3x3 globalRotation_rh = cameraRotation_rh * rotation270degXZ;
    tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;
    tf::Transform transform = tf::Transform(globalRotation_rh, globalTranslation_rh);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "orb_map_", "orb_pose_"));
*/
	
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
}


