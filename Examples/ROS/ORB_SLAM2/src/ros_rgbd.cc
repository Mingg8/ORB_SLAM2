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
#include <python2.7/Python.h>


#include"../../../../include/System.h"
using namespace std;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* _SLAM, bool _mapInitiation):SLAM(_SLAM), mapInitiation(_mapInitiation){}
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped& amcl_pose);
    void Initialize();

    ORB_SLAM2::System* SLAM ;
    ros::NodeHandle private_nh;
    bool mapInitiation;
    geometry_msgs::Pose pose_msg;
    ofstream f_orb_pose;
    ofstream f_amcl_pose;

    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    message_filters::Subscriber<sensor_msgs::Image>* rgb_sub;
    message_filters::Subscriber<sensor_msgs::Image>* depth_sub;
    message_filters::Synchronizer<sync_pol>* sync;

    std::vector<geometry_msgs::PoseWithCovarianceStamped> amcl_vec;


};


void ImageGrabber::Initialize()
{
    pose_pub = nh.advertise<geometry_msgs::Pose>("/orb_pose", 30);
    rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/astra/rgb/image_raw", 1);
    depth_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "astra/depth/image_raw", 1);

    sync = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *rgb_sub, *depth_sub);
    sync->registerCallback(boost::bind(&ImageGrabber::GrabRGBD, this, _1, _2));

    if (mapInitiation){
        ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1000, &ImageGrabber::amclCallback, this);
        //amcl, orb vector initialize
    }


    //FOR DEBUGGING
    f_orb_pose.open("orb_pose_output.txt");
    f_amcl_pose.open("amcl_pose_output.txt");

    ros::spin();

    SLAM->Shutdown();
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return ;
}

void ImageGrabber::amclCallback(const geometry_msgs::PoseWithCovarianceStamped& amcl_pose)
{
//    amcl_vec.push_back(amcl_pose);
    f_amcl_pose << setprecision(6) << amcl_pose.header.stamp << setprecision(7) << " " << amcl_pose.pose.pose.position.x << " "<< amcl_pose.pose.pose.position.y
        << " "<< amcl_pose.pose.pose.position.z << " " <<amcl_pose.pose.pose.orientation.x<<" "<<amcl_pose.pose.pose.orientation.y <<" "
        <<amcl_pose.pose.pose.orientation.z << " "<<amcl_pose.pose.pose.orientation.w << endl;

}


int main(int argc, char **argv)
{
ros::init(argc, argv, "RGBD");
ros::start();

//==========================
bool rosrun = false;

int argv_ind;
if (rosrun){
argv_ind = 2;

}
else{
argv_ind = 1;
}
//==========================

bool mapInitiation = false;
bool saveMapfile = true;

//Check settings file
std::cout<<"file name: "<< (string)argv[argv_ind]<<std::endl;
cv::FileStorage fsSettings((string)argv[argv_ind], cv::FileStorage::READ);
if(!fsSettings.isOpened())
{
   cerr << "Failed to open settings file at: " << argv[argv_ind] << endl;
   exit(-1);
}

cv::FileNode loadmapfilen = fsSettings["Map.loadMapfile"];
string loadmapfile = (string)loadmapfilen;
std::cout<<"loadmapfile: "<<loadmapfile<<std::endl;
std::ifstream in(loadmapfile, std::ios_base::binary);

if (!in){
cerr << "Cannot Open Mapfile: " << loadmapfile << " , You need create it first" << std::endl;
mapInitiation = true;

//amcl callback register
}

cv:: FileNode vocfilen = fsSettings["Map.OrbVoc"];
string vocfile = (string)vocfilen;

//std::map<std::string, std::string> map;
//ros::NodeHandle private_nh;
//private_nh.getParam(private_nh("~"), map);

//ros::NodeHandle nh;
//string vocfile;
//std::vector<string> node_names_;
//XmlRpc::XmlRpcValue v;
//nh.param("ORB", v,v);
//
//for(int i =0; i<v.size(); i++)
//{
//    node_names_.push_back(v[i]);
//    std::cerr<<"node_names: "<< node_names_[i]<<std::endl;
//}

std::map<std::string, std::string> map;
ORB_SLAM2::System mainSLAM(vocfile, argv[argv_ind], map, ORB_SLAM2::System::RGBD, true, saveMapfile);
ImageGrabber igb(&mainSLAM, mapInitiation);
igb.Initialize();

//mapInitiation=false;
//if(!mapInitiation)
//{
//    Py_Initialize();
//    PyObject *pName, *pModule, *pDict, *pFunc, *pArgs, *pValue;
//    // Convert the file name to a Python string.
//    pName = PyString_FromString("orb_map_calibration.py");
//    if (pName==NULL)
//     printf("file not found");
//    else
//    printf("%s\n", PyString_AsString(pName));
//    // Import the file as a Python module.
//    pModule = PyImport_Import(pName);         //PROBLEM LINE
//    if(pModule==NULL)
//     printf("no Module\n");
//    // Create a dictionary for the contents of the module.
//    pDict = PyModule_GetDict(pModule);
//    printf("After Dictionary retrieval\n");
//    // Get the add method from the dictionary.
//    pFunc = PyDict_GetItemString(pDict, "square");
//    printf("after function retrieval\n");
//
//    // Convert 2 to a Python integer.
//    pValue = PyInt_FromLong(2);
//    // Call the function with the arguments.
//    PyObject* pResult = PyObject_CallObject(pFunc, pValue);
//    // Print a message if calling the method failed.
//    if(pResult == NULL)
//    printf("Calling the add method failed.\n");
//    // Convert the result to a long from a Python object.
//    long result = PyInt_AsLong(pResult);
//    // Destroy the Python interpreter.
//    Py_Finalize();
//    // Print the result.
//    printf("The result is %d.\n", result);
//}

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

    pose_msg.position.x=transform.getOrigin().x();
    pose_msg.position.y=transform.getOrigin().y();
    pose_msg.position.z=transform.getOrigin().z();
    pose_msg.orientation.x=transform.getRotation().x();
    pose_msg.orientation.y=transform.getRotation().y();
    pose_msg.orientation.z=transform.getRotation().z();
    pose_msg.orientation.w=transform.getRotation().w();

    pose_pub.publish(pose_msg);
    f_orb_pose << fixed;
    f_orb_pose << setprecision(6) << cv_ptrRGB->header.stamp.toSec() << setprecision(7) <<" "<< pose_msg.position.x << " "<< pose_msg.position.y << " "<< pose_msg.position.z << " "<<pose_msg.orientation.x<<" "<<pose_msg.orientation.y <<" " <<pose_msg.orientation.z << " "<<pose_msg.orientation.w <<endl;

}
