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
#include <nav_msgs/OccupancyGrid.h>

#include <math.h>
#include <vector>
#include <signal.h>
#include <visualization_msgs/Marker.h>

#include "pepper_obj_msgs/objs_array.h"
#include "pepper_obj_msgs/objs.h"
#include <cmath>

double orb_scale = 1.0;
volatile sig_atomic_t flag = 0;
vector<double> orb_trans;
vector<double> orb_quat;
tf::Transform map_transform;
float l = 0.0;
float r = 0.0;
float u = 0.0;
float b = 0.0;
bool first_flag = true;

int threshold = 2;
float size = 0.01;

ORB_SLAM2::Map* mpMap;

const int map_update_rate = 3;
int update_counter = map_update_rate;

const int map_update_rate_2d = 10;
int update_counter_2d = map_update_rate_2d;

typedef boost::shared_ptr< ::pepper_obj_msgs::objs_array const> ObjConstPtr;

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const ObjConstPtr& msgC,
			 ros::NodeHandle& nh, ros::Publisher& marker_pub, ros::Publisher& map_pub);

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

  if(!first_flag) {
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
    first_flag = false;
  }
}

void make_map(std::vector<std::vector<int>>& map_matrix, float x, float y, float z)
{
  tf::Vector3 pose(x*orb_scale, z*orb_scale, y*orb_scale);
  tf::Vector3 transformed_pose = map_transform * pose;

  if (transformed_pose.getZ() > 1.5) return;

  geometry_msgs::Point p;
  int map_x = (int) ((transformed_pose.getX() - l) / size);
  int map_y = (int) ((transformed_pose.getY() - b) / size);
//  std::cout << map_x << ", " << map_y << std::endl;

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

void publish_map(nav_msgs::OccupancyGrid& grid_map, std::vector<std::vector<int>>& map_matrix, int rows, int cols)
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
			      if(map_matrix[i][j] >= threshold){
			        	grid_map.data[index] = 100;
			      }
			      else{
			        	grid_map.data[index] = 0;
			      }
			      index++;
			  }
	  }
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
    cv::rectangle(img, rc, color, CV_FILLED);  // filled rectangle
  }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "MAP");
    ros::start();
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker> ("map_marker", 10);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid> ("/map", 10);
    ros::Rate rate(1000);

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

    /*visualization_msgs::Marker map;
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
    map.color.a = 1.0;*/

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

    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    orb_scale = (double) fsSettings["Map.scale"];

    cv::FileNode map_t_ = fsSettings["Map.translation"];
    cv::FileNode map_q_ = fsSettings["Map.quaternion"];

    for(cv::FileNodeIterator it = map_t_.begin(); it!=map_t_.end(); ++it)
        orb_trans.push_back((double)*it);

    for(cv::FileNodeIterator it = map_q_.begin(); it!=map_q_.end(); ++it)
        orb_quat.push_back((double)*it);

    nh.setParam("orb_translation", orb_trans);
    nh.setParam("orb_quaternion", orb_quat);


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true, true);
    mpMap = SLAM.getMap();

    ImageGrabber igb(&SLAM);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/pepper_robot/camera/front/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/pepper_robot/camera/depth/image_raw", 1);
    message_filters::Subscriber<pepper_obj_msgs::objs_array> caption_sub(nh, "/objects", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, pepper_obj_msgs::objs_array> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub,caption_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2,_3,nh,marker_pub,map_pub));

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

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,
	 const ObjConstPtr& msgC, ros::NodeHandle& nh, ros::Publisher& marker_pub, ros::Publisher& map_pub)
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

    if(update_counter++ == map_update_rate) {
        visualization_msgs::Marker points;
        get_map_points(points);
        points.header.stamp = ros::Time::now();
        marker_pub.publish(points);
        update_counter = 1;
    }

    /////////////////////////////////

    if(update_counter_2d++ == map_update_rate_2d) {
				nav_msgs::OccupancyGrid grid_map;
        const vector<ORB_SLAM2::MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
        const vector<ORB_SLAM2::MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();
        set<ORB_SLAM2::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if(vpMPs.empty())
            return;

				/*u-=23;
				b+=25;
				r-=10;
				l+=6;*/
				int cols = (int)((r-l) / size)+1;
		    int rows = (int)((u-b) / size)+1;
		    std::cout << "map rows: " << rows << " | cols: " << cols << std::endl;
		    std::vector<std::vector<int>> map_matrix(rows, std::vector<int>(cols));

				for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
		    {
		        if(vpMPs[i]->isBad())
		            continue;
		        cv::Mat pos = vpMPs[i]->GetWorldPos();
		        make_map(map_matrix, pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
		    }

		    publish_map(grid_map, map_matrix, rows, cols);

				grid_map.header.stamp = ros::Time::now();
        map_pub.publish(grid_map);
				update_counter_2d = 1;
				//std::cout << grid_map.info.origin.position << std::endl;
    }
}
