#include <python2.7/Python.h>
#include<algorithm>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include "RANSAC.h"

using namespace Eigen;

namespace ORB_SLAM2
{
    RANSAC::RANSAC(std::vector<std::vector<float>>& _orb_poses, std::vector<std::vector<float>>& _amcl_poses): raw_orb_poses(_orb_poses), raw_amcl_poses(_amcl_poses){
        itr_num = 300;
        threshold = 0.1;
        time_interval = 10;
        tf_compute_interval = 10;;
        load_data(raw_orb_poses, raw_amcl_poses);
        fitting();
    }

    void RANSAC::load_data(std::vector<std::vector<float>>& raw_orb_poses, std::vector<std::vector<float>>& raw_amcl_poses){
        std::vector<std::vector<float>> raw_poses_1 = raw_orb_poses;
        std::vector<std::vector<float>> raw_poses_2 = raw_amcl_poses;
        std::vector<std::vector<float>> poses_1, poses_2;
        int idx_1=0;

        raw_poses_1 = (raw_orb_poses.size()>= raw_amcl_poses.size())? raw_orb_poses: raw_amcl_poses;
        raw_poses_2 = (raw_orb_poses.size()>= raw_amcl_poses.size())? raw_amcl_poses : raw_orb_poses;

        if(idx_1 <raw_poses_1.size()){
            for(int idx_2=0; idx_2< raw_poses_2.size(); idx_2++){
                std::vector<float>pose_2 = raw_poses_2.at(idx_2);
                float time_diff = pose_2.at(0) - (raw_poses_1.at(idx_1)).at(0);

                //find most similar time stamp of each orb_pose
                while (time_diff>=0){
                    if (idx_1 < raw_poses_1.size()){
                        idx_1++;
                        time_diff = pose_2.at(0) - (raw_poses_1.at(idx_1)).at(0);
                    }
                    else
                        break;
                }
                if (idx_1 >=1 && abs(time_diff)>= abs(pose_2.at(0)-(raw_poses_1.at(idx_1-1)).at(0))){
                    idx_1 --;
                    time_diff = pose_2.at(0) - (raw_poses_1.at(idx_1)).at(0);
                }

                //check if the datas are adequate
                if (abs(time_diff) <= time_interval){
                    poses_1.push_back(raw_poses_1.at(idx_1));
                    poses_2.push_back(pose_2);
                    idx_1++;
                }
            }
        }
        orb_poses = (raw_orb_poses.size() >= raw_amcl_poses.size()) ? poses_1 : poses_2;
        amcl_poses = (raw_orb_poses.size() >= raw_amcl_poses.size()) ? poses_2 : poses_1;
        if (amcl_poses.size() != orb_poses.size())
            std::cout<< "amcl size: " << amcl_poses.size() << ", orb size: "<< orb_poses.size() << ", Sth is wrong!!" << std::endl;
        else
            std::cout<<"datas loaded, size: "<< amcl_poses.size() <<std::endl;
    }

    void RANSAC::fitting(){
        int c_max = 0;
        mtx_opt = Matrix4f::Identity();
        int n_data= amcl_poses.size();
        for (int itr=0; itr<=itr_num; itr++){
            if(n_data ==0){
                std::cout << "no data" << std::endl;
                break;
            }
            else{
                int k = (int)floor(rand()%n_data);
                std::vector<float> orb_pose = orb_poses.at(k);
                std::vector<float> amcl_pose = amcl_poses.at(k);
                Quaternion<float> q_orb(orb_pose.at(3), orb_pose.at(4), orb_pose.at(5), orb_pose.at(6));
                Matrix3f R_orb = q_orb.normalized().toRotationMatrix();
                Matrix4f orb_trans;
                orb_trans << R_orb(0,0), R_orb(0,1), R_orb(0,2), orb_pose.at(0),
                                        R_orb(1,0), R_orb(1,1), R_orb(1,2), orb_pose.at(1),
                                        R_orb(2,0), R_orb(2,1), R_orb(2,2), orb_pose.at(2),
                                        0, 0, 0, 1;

                Eigen::Quaternion<float> q_amcl(amcl_pose.at(3), amcl_pose.at(4), amcl_pose.at(5), amcl_pose.at(6));
                Eigen::Matrix3f R_amcl = q_amcl.normalized().toRotationMatrix();
                Eigen::Matrix4f amcl_trans;
                amcl_trans << R_amcl(0,0), R_amcl(0,1), R_amcl(0,2), amcl_pose.at(0),
                                        R_amcl(1,0), R_amcl(1,1), R_amcl(1,2), amcl_pose.at(1),
                                        R_amcl(2,0), R_amcl(2,1), R_amcl(2,2), amcl_pose.at(2),
                                        0, 0, 0, 1;

                Eigen::Matrix4f mtx = amcl_trans * orb_trans.inverse();
                int cnt = 0;
                for(int i=0; i<=(int)orb_poses.size(); i++){
                    if(error(orb_trans, amcl_trans, mtx) < threshold)
                        cnt ++;
                    if (cnt > c_max){
                        c_max = cnt;
                        mtx_opt = mtx;
                    }
                }
            }
        }
    }

    float RANSAC::error(Eigen::Matrix4f& orb_trans, Eigen::Matrix4f& amcl_trans, Eigen::Matrix4f& mtx){
        Eigen::Matrix4f pred_amcl_trans = mtx * orb_trans;

        float err = sqrt(pow(amcl_trans(0,3)-pred_amcl_trans(0,3),2)+pow(amcl_trans(1,3)-pred_amcl_trans(1,3),2)+pow(amcl_trans(2,3)-pred_amcl_trans(2,3),2));
        return err;
    }

    Eigen::Matrix4f* RANSAC::getMtx(){
    return &mtx_opt;
    }
} //namespace ORB_SLAM
