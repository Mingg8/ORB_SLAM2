#include <python2.7/Python.h>
#include <algorithm>
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
    RANSAC::RANSAC(std::vector<std::vector<float>>& _orb_poses, std::vector<std::vector<float>>& _amcl_poses, int _rsc_itr_num, float _rsc_thres, float _time_interval)
                : raw_orb_poses(_orb_poses), raw_amcl_poses(_amcl_poses), rsc_itr_num(_rsc_itr_num), rsc_thresh(_rsc_thres), time_interval(_time_interval){
        load_data(raw_orb_poses, raw_amcl_poses);
        fitting();
    }

    // Find datas that timestamp matches from raw_orb_poses and raw_amcl_poses each
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
                    if (idx_1 < raw_poses_1.size()-1){
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
        for(int itr=0; itr<=rsc_itr_num; itr++){
            if(n_data ==0){
                std::cout << "no data" << std::endl;
                break;
            }
            else{
                int k = (int)floor(rand()%n_data);
                std::vector<float> orb_pose = orb_poses.at(k);
                std::vector<float> amcl_pose = amcl_poses.at(k);
                Matrix4f orb_trans;
                Eigen::Matrix4f amcl_trans;
                vec2mat(orb_pose, orb_trans);
                vec2mat(amcl_pose, amcl_trans);
                Eigen::Matrix4f mtx = amcl_trans * orb_trans.inverse();
                int cnt = 0;
                for(int i=0; i<(int)orb_poses.size(); i++){
                    Eigen::Matrix4f orb_mat, amcl_mat;
                    vec2mat(orb_poses.at(i), orb_mat);
                    vec2mat(amcl_poses.at(i), amcl_mat);
                    // std::cout<<orb_mat<<amcl_mat<<mtx<<std::endl;
                    float err = error(orb_mat, amcl_mat, mtx);
                    if(err<rsc_thresh){
                        cnt++;
                    }
                }
                if (cnt > c_max){
                    c_max = cnt;
                    mtx_opt = mtx;
                }
            }
        }
    }


    float RANSAC::error(Eigen::Matrix4f& _orb_mat, Eigen::Matrix4f& _amcl_mat, Eigen::Matrix4f& _mtx){
        Eigen::Matrix4f pred_amcl_trans = _mtx * _orb_mat;

        float err = sqrt(pow(_amcl_mat(0,3)-pred_amcl_trans(0,3),2)+pow(_amcl_mat(1,3)-pred_amcl_trans(1,3),2)+pow(_amcl_mat(2,3)-pred_amcl_trans(2,3),2));
        return err;
    }

    void RANSAC::vec2mat(std::vector<float>& vec, Eigen::Matrix4f& Mat){
        Quaternion<float> q(vec.at(3), vec.at(4), vec.at(5), vec.at(6));
        Matrix3f Rot = q.normalized().toRotationMatrix();
        Mat << Rot(0,0), Rot(0,1), Rot(0,2), vec.at(0),
                Rot(1,0), Rot(1,1), Rot(1,2), vec.at(1),
                Rot(2,0), Rot(2,1), Rot(2,2), vec.at(2),
                0, 0, 0, 1;
    }
    void RANSAC::getMtx(Eigen::Matrix4f& mtx){
    mtx = mtx_opt;
    }
} //namespace ORB_SLAM
