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

#include <ros/ros.h>
#include <rosbag/bag.h>
//#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include "opencv2/ccalib/omnidir.hpp"

#include"System.h"
#include"Converter.h"
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <algorithm> //boolalpha

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    tf::Transform GrabImage(cv::Mat& frame, tf::Transform& pose_previous, double& time_stamp);

    ORB_SLAM2::System* mpSLAM;
};

nav_msgs::Odometry create_odometry_message(tf::Transform& transform, ros::Time& time_stamp){
	//creates a ros odometry message
	
	nav_msgs::Odometry msg_odom;
	msg_odom.header.stamp = time_stamp;
	msg_odom.header.frame_id = "odom";
	msg_odom.header.seq = 0;
	msg_odom.child_frame_id = "odom_link";

	//msg_odom.pose.pose.position = transform.getOrigin();
	msg_odom.pose.pose.position.x = transform.getOrigin().getX();
        msg_odom.pose.pose.position.y = transform.getOrigin().getY();
        msg_odom.pose.pose.position.z = transform.getOrigin().getZ();
	
	//msg_odom.pose.pose.orientation = transform.getRotation();
	//normalize ()
	msg_odom.pose.pose.orientation.x = transform.getRotation().x();
	msg_odom.pose.pose.orientation.y = transform.getRotation().y();
	msg_odom.pose.pose.orientation.z = transform.getRotation().z();
	msg_odom.pose.pose.orientation.w = transform.getRotation().w();

	/*msg_odom.pose.covariance[0] = std::pow(ins_data->PosUncertainty,2.0);
	msg_odom.pose.covariance[7] = std::pow(ins_data->PosUncertainty,2.0);
	msg_odom.pose.covariance[14] = std::pow(ins_data->PosUncertainty,2.0);
	msg_odom.pose.covariance[21] = std::pow(ins_data->RollUncertainty,2.0);
	msg_odom.pose.covariance[28] = std::pow(ins_data->PitchUncertainty,2.0);
	msg_odom.pose.covariance[35] = std::pow(ins_data->YawUncertainty,2.0);*/
	return msg_odom;
}

bool to_bool(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    std::istringstream is(str);
    bool b;
    is >> std::boolalpha >> b;
    return b;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings slam_config_file_path" << endl;        
        ros::shutdown();
        return 1;
    }    
	//read config
	std::ifstream file(argv[3]);
	std::string data_folder_path, video_path, line;
	std::getline(file, data_folder_path);
	std::getline(file, video_path);
	std::getline(file, line);
	double first_time_stamp = std::stod(line);
	std::getline(file, line);
	double fps = std::stod(line);
	double frame_duration = 1.0/fps;
	std::getline(file, line);
	bool resize_bool = to_bool(line);
	//ROS_INFO_STREAM(line);

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

	ImageGrabber igb(&SLAM);
	//ros::NodeHandle nodeHandler;

	//open video
	cv::VideoCapture capture(video_path);
	if( !capture.isOpened() )
        	throw "Error when reading video";

	//read calibration
	/*cv::FileStorage fs("/home/silvere/Downloads/000-000-viljandi/0012/out_camera_params.xml", cv::FileStorage::READ);
	cv::Mat K, xi, D;
	fs["camera_matrix"] >> K;
	fs["distortion_coefficients"] >> D;
	fs["xi"] >> xi;*/
	
	//open ros bag
	rosbag::Bag bag;
	bag.open(data_folder_path + "odometry.bag", rosbag::bagmode::Write);

	//variables
	cv::Mat frame;
	bool publish=true;
	tf::Transform pose_previous;
	pose_previous.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
    	tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
    	pose_previous.setRotation(q);
	nav_msgs::Odometry odometry_msg;
	double frame_time = first_time_stamp;
	//start tracking
	if(publish){
		//ros::Publisher slam_pub = n.advertise<tf::Transform>("mono", 1000);
		while (ros::ok()){
			//read image
			capture >> frame;
			if(frame.size().width < 1){
				break;
				frame_time = frame_time + frame_duration;
			}
			//
			//resize image
			if(resize_bool){
				cv::resize(frame, frame, cv::Size(), 0.5, 0.5);
			}
			//track slam
			tf::Transform pose = igb.GrabImage(frame, pose_previous, frame_time);
			//create odometry message from pose and time stamp
			ros::Time time_stamp(frame_time);
			odometry_msg = create_odometry_message(pose, time_stamp);
			//ROS_INFO_STREAM(odometry_msg.pose.pose.position);
			bag.write("odom", time_stamp, odometry_msg);
			// Publish tf transform
			static tf::TransformBroadcaster br;
			br.sendTransform(tf::StampedTransform(pose, time_stamp, "camera", "world")); // camera is the parent frame
			//slam_pub.publish(tf_msg);
			ros::spinOnce();
			frame_time = frame_time + frame_duration;
		}
	}
	else{
		//ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
	}

    //ros::spin();
	bag.close();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(data_folder_path + "KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM(data_folder_path + "FrameTrajectory.txt");
    //SLAM.SaveTrajectoryTUM2(data_folder_path + "FrameTrajectory2.txt");
    //SLAM.SaveTrajectoryKITTI(data_folder_path + "FrameTrajectoryKITTI.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec()).clone();
    if(Tcw.empty())
        return;
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    // create transform
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(tcw.at<float>(0), tcw.at<float>(1), tcw.at<float>(2)));
    vector<float> r = ORB_SLAM2::Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
    tf::Quaternion q(r[0], r[1], r[2], r[3]);
    transform.setRotation(q);
    // Publish tf transform
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "world")); // camera is the parent frame
}

tf::Transform ImageGrabber::GrabImage(cv::Mat& frame, tf::Transform& pose_previous, double& time_Stamp)
{
    //save old pose
    //pose_previous = pose;

    //slam frame
    cv::Mat Tcw = mpSLAM->TrackMonocular(frame, time_Stamp).clone();
    if(Tcw.empty())
        return pose_previous;
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);

	//ROS_INFO_STREAM(Tcw);
	//ROS_INFO_STREAM(mpSLAM->mpTracker->mCurrentFrame.mTcw);

    // Create tf transform
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(tcw.at<float>(0), tcw.at<float>(1), tcw.at<float>(2)));
    vector<float> r = ORB_SLAM2::Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
    tf::Quaternion q(r[0], r[1], r[2], r[3]);
    transform.setRotation(q);
    return transform;
}


