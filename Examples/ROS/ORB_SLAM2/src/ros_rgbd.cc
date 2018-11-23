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
//TODO: change every path to relative path (orb.yaml / launch file)
//TODO: pass amcl vec, orb vec, data.xml path to python file
//TODO: revise launch file
//TODO: export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/mjlee/ws/src/external_ros/ORB_SLAM2/Examples/ROS

#include <python2.7/Python.h>
#include<algorithm>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include"../../../../include/System.h"
#include"../../../../include/RANSAC.h"
using namespace std;
using namespace Eigen;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* _SLAM, bool _mapInitiation, cv::Mat _mtx):SLAM(_SLAM), mapInitiation(_mapInitiation), calibMtx(_mtx){}
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    std::vector<std::vector<float>> amcl_poses;
    std::vector<std::vector<float>> orb_poses;
    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped& amcl_pose);

    void Initialize();

    ORB_SLAM2::System* SLAM ;
    ros::NodeHandle private_nh;
    bool mapInitiation;
    cv::Mat calibMtx;
    geometry_msgs::Pose pose_msg;
//    ofstream f_orb_pose;
//    ofstream f_amcl_pose;

    int image_grab_count = 0;

    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    message_filters::Subscriber<sensor_msgs::Image>* rgb_sub;
    message_filters::Subscriber<sensor_msgs::Image>* depth_sub;
    message_filters::Synchronizer<sync_pol>* sync;

//    vector<geometry_msgs::PoseWithCovarianceStamped> amcl_vec;
};


void ImageGrabber::Initialize()
{
    pose_pub = nh.advertise<geometry_msgs::Pose>("/orb_pose", 30);
    rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/astra/rgb/image_raw", 1);
    depth_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "astra/depth/image_raw", 1);

    sync = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *rgb_sub, *depth_sub);
    sync->registerCallback(boost::bind(&ImageGrabber::GrabRGBD, this, _1, _2));

    ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1000, &ImageGrabber::amclCallback, this);
    if (mapInitiation){
//        ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1000, &ImageGrabber::amclCallback, this);
//        f_orb_pose.open("/home/mjlee/ws/src/external_ros/ORB_SLAM2/orb_pose_output.txt");
//        f_amcl_pose.open("/home/mjlee/ws/src/external_ros/ORB_SLAM2/amcl_pose_output.txt");
        //amcl, orb vector initialize
    }

    ros::spin();

    SLAM->Shutdown();
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
//    f_orb_pose.close();
//    f_amcl_pose.close();

    return ;
}

void ImageGrabber::amclCallback(const geometry_msgs::PoseWithCovarianceStamped& amcl_pose)
{
    std::vector<float> amcl_vec;
    amcl_vec.push_back(amcl_pose.header.stamp.toSec());
    amcl_vec.push_back(amcl_pose.pose.pose.position.x);
    amcl_vec.push_back(amcl_pose.pose.pose.position.y);
    amcl_vec.push_back(amcl_pose.pose.pose.position.z);

    amcl_vec.push_back(amcl_pose.pose.pose.orientation.x);
    amcl_vec.push_back(amcl_pose.pose.pose.orientation.y);
    amcl_vec.push_back(amcl_pose.pose.pose.orientation.z);
    amcl_vec.push_back(amcl_pose.pose.pose.orientation.w);

    amcl_poses.push_back(amcl_vec);

//    f_amcl_pose << setprecision(6) << amcl_pose.header.stamp << setprecision(7) << " " << amcl_pose.pose.pose.position.x << " "<< amcl_pose.pose.pose.position.y
//        << " "<< amcl_pose.pose.pose.position.z << " " <<amcl_pose.pose.pose.orientation.x<<" "<<amcl_pose.pose.pose.orientation.y <<" "
//        <<amcl_pose.pose.pose.orientation.z << " "<<amcl_pose.pose.pose.orientation.w << endl;

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

    cv::Mat pose = SLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());


    if (pose.empty())
        return;

    static cv::Mat pose_prev = cv::Mat::eye(4, 4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4, 4, CV_32F);
    static const cv::Mat flipSign = (cv::Mat_<float>(4,4) << 1,-1, -1, 1,
                                                            -1, 1, -1, 1,
                                                            -1, -1, 1, 1,
                                                            1, 1, 1, 1);

    cv::Mat translation = (pose* pose_prev.inv()).mul(flipSign);
    world_lh = world_lh * translation;
    pose_prev = pose.clone();

    tf::Matrix3x3 cameraRotation_rh( -world_lh.at<float>(0,0), world_lh.at<float>(0,1), world_lh.at<float>(0,2),
                                    -world_lh.at<float>(1,0), world_lh.at<float>(1,1), world_lh.at<float>(1,2),
                                    world_lh.at<float>(2,0), -world_lh.at<float>(2,1), -world_lh.at<float>(2,2));

    tf::Vector3 cameraTranslation(world_lh.at<float>(0,3), world_lh.at<float>(1,3), -world_lh.at<float>(2,3));


    const tf::Matrix3x3 rotation270degXZ( 0, 1, 0,
                                        0, 0, 1,
                                        1, 0, 0);

    tf::Matrix3x3 globalRotation_rh = cameraRotation_rh * rotation270degXZ;
    tf::Vector3 globalTranslation_rh = cameraTranslation * rotation270degXZ;
    tf::Transform transform = tf::Transform(globalRotation_rh, globalTranslation_rh);

    if(!mapInitiation){

        Matrix4f eigMtx;
        cv::cv2eigen(calibMtx, eigMtx);

        Matrix4f poseMtx;
        poseMtx << globalRotation_rh.getRow(0).getX() , globalRotation_rh.getRow(0).getY(), globalRotation_rh.getRow(0).getZ(), globalTranslation_rh.getX(),
                    globalRotation_rh.getRow(1).getX(), globalRotation_rh.getRow(1).getY(), globalRotation_rh.getRow(1).getZ(), globalTranslation_rh.getY(),
                    globalRotation_rh.getRow(2).getX(), globalRotation_rh.getRow(2).getY(), globalRotation_rh.getRow(2).getZ(), globalTranslation_rh.getZ(),
                    0, 0, 0, 1;
        Matrix4f transformedPose = eigMtx * poseMtx;
        tf::Vector3 origin;
        origin.setValue(static_cast<double>(transformedPose(0,3)),static_cast<double>(transformedPose(1,3)),static_cast<double>(transformedPose(2,3)));
        tf::Matrix3x3 tf3d;
        tf3d.setValue(static_cast<double>(transformedPose(0,0)), static_cast<double>(transformedPose(0,1)), static_cast<double>(transformedPose(0,2)),
            static_cast<double>(transformedPose(1,0)), static_cast<double>(transformedPose(1,1)), static_cast<double>(transformedPose(1,2)),
            static_cast<double>(transformedPose(2,0)), static_cast<double>(transformedPose(2,1)), static_cast<double>(transformedPose(2,2)));

        transform=tf::Transform(tf3d, origin);
    }
    else
        transform = tf::Transform(globalRotation_rh, globalTranslation_rh);

    pose_msg.position.x=transform.getOrigin().x();
    pose_msg.position.y=transform.getOrigin().y();
    pose_msg.position.z=transform.getOrigin().z();
    pose_msg.orientation.x=transform.getRotation().x();
    pose_msg.orientation.y=transform.getRotation().y();
    pose_msg.orientation.z=transform.getRotation().z();
    pose_msg.orientation.w=transform.getRotation().w();

    if(mapInitiation){
        std::vector<float> orb_pose;
        orb_pose.push_back(cv_ptrRGB->header.stamp.toSec());
        orb_pose.push_back(pose_msg.position.x);
        orb_pose.push_back(pose_msg.position.y);
        orb_pose.push_back(pose_msg.position.z);
        orb_pose.push_back(pose_msg.orientation.x);
        orb_pose.push_back(pose_msg.orientation.y);
        orb_pose.push_back(pose_msg.orientation.z);
        orb_pose.push_back(pose_msg.orientation.w);

        orb_poses.push_back(orb_pose);
//        f_orb_pose << fixed;
//        f_orb_pose << setprecision(6) << cv_ptrRGB->header.stamp.toSec() << setprecision(7) <<" "<< pose_msg.position.x << " "<< pose_msg.position.y << " "<< pose_msg.position.z << " "<<pose_msg.orientation.x<<" "<<pose_msg.orientation.y <<" " <<pose_msg.orientation.z << " "<<pose_msg.orientation.w <<endl;
    }
    else
        pose_pub.publish(pose_msg);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    int argv_ind = 1;

    bool mapInitiation = false;
    bool saveMapfile = true;

    //Check settings file
    cout<<"file name: "<< (string)argv[argv_ind]<<endl;
    cv::FileStorage fsSettings((string)argv[argv_ind], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << argv[argv_ind] << endl;
       exit(-1);
    }

    //TODO: Load setting file as vector(?) from yaml file or use absolute path
    //TODO: Code Refactoring

    cv::FileNode loadmapfilen = fsSettings["Map.loadMapfile"];
    string loadmapfile = (string)loadmapfilen;
    cout<<"loadmapfile: "<<loadmapfile<<endl;
    ifstream in(loadmapfile, ios_base::binary);

    cv::FileNode calibfilen = fsSettings["Map.CalibMtx"];
    string calibfile = (string)calibfilen;
    cv::FileStorage calib_fs(calibfilen, cv::FileStorage::READ);


    //If calibration matrix or map file doesn't exist, initiate map
    if (!in.is_open() || !calib_fs.isOpened()){
    mapInitiation = true;
    }

    cout << "Map loading " << in.is_open() << endl;
    cout << "calibration matrix loading " << calib_fs.isOpened() << endl;

    cv::Mat mtx;
    if (mapInitiation){
    }
    else{
        cout << "Load calibration file" <<endl;
        calib_fs["data"] >> mtx;
    }

    cout<<"Map initialization: "<< mapInitiation <<endl;
    cv:: FileNode vocfilen = fsSettings["Map.OrbVoc"];
    string vocfile = (string)vocfilen;

    saveMapfile = false;
    ORB_SLAM2::System mainSLAM(vocfile, argv[argv_ind], ORB_SLAM2::System::RGBD, true, saveMapfile);
    ImageGrabber igb(&mainSLAM, mapInitiation, mtx);
    igb.Initialize();

    //Calculate calibration matrix from orb_pose and amcl_pose
    if(mapInitiation)
    {
        ORB_SLAM2::RANSAC rsc(igb.orb_poses, igb.amcl_poses);
        Eigen::Matrix4f* calib_mtx;
        calib_mtx = rsc.getMtx();
        cv::FileStorage cv_file("/home/mjlee/ws/src/external_ros/ORB_SLAM2/data.xml", cv::FileStorage::WRITE);
        cv_file.writeObj("data", &calib_mtx);
        cv_file.release();
    }
}