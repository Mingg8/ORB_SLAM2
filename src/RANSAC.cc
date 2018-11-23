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
    RANSAC::RANSAC(std::vector<std::vector<float>> _orb_poses, std::vector<std::vector<float>> _amcl_poses): orb_poses(_orb_poses), amcl_poses(_amcl_poses){
        n_data = amcl_poses.size();
        itr_num = 300;
        threshold = 0.1;
        fitting();
    }

    void RANSAC::fitting(){
        int c_max = 0;
        mtx_opt = Matrix4f::Identity();
        for (int itr=0; itr<=itr_num; itr++){
            if(n_data ==0){
                std::cout << "no data" << std::endl;
                break;
            }
            else{
                int k = (int)floor(rand()%n_data +1);
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
                    if(error(&(orb_trans), &(amcl_trans), &mtx) < threshold)
                        cnt ++;
                    if (cnt > c_max){
                        c_max = cnt;
                        mtx_opt = mtx;
                    }
                }
            }
        }
    }

    float RANSAC::error(Eigen::Matrix4f* orb_trans, Eigen::Matrix4f* amcl_trans, Eigen::Matrix4f* mtx){
        Eigen::Matrix4f pred_amcl_trans = *mtx * *orb_trans;

        float err = sqrt(pow((*amcl_trans)(0,3)-pred_amcl_trans(0,3),2)+pow((*amcl_trans)(1,3)-pred_amcl_trans(1,3),2)+pow((*amcl_trans)(2,3)-pred_amcl_trans(2,3),2));
        return err;
    }

    Eigen::Matrix4f* RANSAC::getMtx(){
    return &mtx_opt;
    }
} //namespace ORB_SLAM
