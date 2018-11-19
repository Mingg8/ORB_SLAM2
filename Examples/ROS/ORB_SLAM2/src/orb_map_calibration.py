#!/usr/bin/env python
import math
import numpy as np
import random
import cv2
import tf

tf_compute_interval = 10
time_interval = 10


class MapCalibration:
    def __init__(self, amcl_addr, orb_addr):

        self.amcl_pose_addr = amcl_addr
        self.orb_pose_addr = orb_addr

        self.orb_poses = []
        self.amcl_poses = []
        self.load_data()
        mtx = RANSAC(self.orb_poses, self.amcl_poses)

    def load_data(self):
        amcl_line_idx = 0
        orb_line_idx = 0
        amcl_pose = []
        orb_pose = []
        append_idx = 0
        with open(self.amcl_pose_addr, "r") as amcl_f:
            amcl_lines = amcl_f.readlines()
            amcl_datas = [[float(i) for i in amcl_data.split(" ")] for amcl_data in amcl_lines]
            with open(self.orb_pose_addr, "r") as orb_f:
                orb_lines = orb_f.readlines()
                for orb_line in orb_lines:
                    orb_data = [float(i) for i in orb_line.split(" ")]
                    time_diff = amcl_datas[amcl_line_idx][0] - orb_data[0]

                    # find most similar time stamp of each orb_pose
                    while time_diff <= 0:
                        if amcl_line_idx < len(amcl_lines) - 1:
                            amcl_line_idx += 1
                            time_diff = amcl_datas[amcl_line_idx][0] - orb_data[0]
                        else:
                            break
                    if amcl_line_idx >= 1 and (
                            abs(time_diff) >= abs(orb_data[0] - amcl_datas[amcl_line_idx - 1][0])):
                        amcl_line_idx -= 1
                        time_diff = orb_data[0] - amcl_datas[amcl_line_idx][0]
                    if abs(time_diff) <= time_interval:
                        self.orb_poses.append(orb_data[1:])
                        self.amcl_poses.append(amcl_datas[amcl_line_idx][1:])
                        append_idx += 1
                        orb_line_idx += 1
                    else:
                        break
        print("Data Loaded -%d lines" % (append_idx))

    def least_square_method(self, orb_pose, amcl_pose):
        orb_pose = np.array(orb_pose)
        amcl_pose = np.array(amcl_pose)

        A = np.vstack([orb_pose, np.ones((len(orb_pose), len(orb_pose[0])))]).T
        M = np.linalg.lstsq(A, amcl_pose, rcond=None)[0]

    def check_scale():
        for i in range(len(amcl_pose) - tf_compute_interval):
            amcl_dist = math.sqrt(math.pow(amcl_pose[i][0] - amcl_pose[0][0], 2) + math.pow(
                amcl_pose[i][1] - amcl_pose[0][1], 2)
                                  + math.pow(amcl_pose[i][2] - amcl_pose[0][2], 2))
            orb_dist = math.sqrt(
                math.pow(orb_pose[i][0] - orb_pose[0][0], 2) + math.pow(
                    orb_pose[i][1] - orb_pose[0][1], 2)
                + math.pow(orb_pose[i][2] - orb_pose[0][2], 2))
            if float(orb_dist) != 0:
                print("amcl_dist: " + str(amcl_dist) + ", orb_dist: " + str(orb_dist) + ", ratio: " + str(
                    amcl_dist / float(orb_dist)))


class RANSAC:
    def __init__(self, _orb_poses, _amcl_poses):
        self.orb_poses = _orb_poses
        self.amcl_poses = _amcl_poses
        self.n_data = len(self.orb_poses)
        self.itr_num = 300
        self.threshold = 0.1

        mtx_opt = self.fitting()

    def calculate_mtx(self, orb_pose, amcl_pose):
        orb_trans = tf.transformations.translation_matrix(orb_pose[:3])
        orb_rot = tf.transformations.quaternion_matrix(orb_pose[3:])
        orb_mat = np.dot(orb_trans, orb_rot)
        # print(orb_mat)
        amcl_trans = tf.transformations.translation_matrix(amcl_pose[:3])
        amcl_rot = tf.transformations.quaternion_matrix(amcl_pose[3:])
        amcl_mat = np.dot(amcl_trans, amcl_rot)

        transform = np.matmul(amcl_mat, np.linalg.inv(orb_mat))

        return transform

    def error(self, orb_pose, amcl_pose, mtx):
        orb_trans = tf.transformations.translation_matrix(orb_pose[:3])
        orb_rot = tf.transformations.quaternion_matrix(orb_pose[3:])
        orb_mat = np.dot(orb_trans, orb_rot)

        amcl_trans = tf.transformations.translation_matrix(amcl_pose[:3])
        amcl_rot = tf.transformations.quaternion_matrix(amcl_pose[3:])
        amcl_mat = np.dot(amcl_trans, amcl_rot)

        pred_amcl_mat = np.matmul(mtx, orb_mat)

        err = math.sqrt(
            math.pow(amcl_mat[0][3] - pred_amcl_mat[0][3], 2) + math.pow(amcl_mat[1][3] - pred_amcl_mat[1][3], 2)
            + math.pow(amcl_mat[2][3] - pred_amcl_mat[2][3], 2))
        return err

    def fitting(self):
        c_max = 0
        mtx_opt = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        print("n_data: " + str(self.n_data))
        for itr in range(self.itr_num):
            if self.n_data == 0:
                print("no data")
                # return
                break
            else:
                # random sampling
                k = int(math.floor(self.n_data * random.random()))
                # print("k: %d" %(k))
                # model estimation
                mtx = self.calculate_mtx(self.orb_poses[k], self.amcl_poses[k])
                cnt = 0
                for (i, (orb_pose, amcl_pose)) in enumerate(zip(self.orb_poses, self.amcl_poses)):
                    if self.error(orb_pose, amcl_pose, mtx) < self.threshold:
                        cnt += 1
                    # print("cnt: %d" %(cnt))
                    if cnt > c_max:
                        c_max = cnt
                        mtx_opt = mtx
                # print("c_max: %d" %(c_max))

        cv_file = cv2.FileStorage("/home/mjlee/ws/src/external_ros/ORB_SLAM2/data.xml", cv2.FILE_STORAGE_WRITE)
        mtx_opt = np.array(mtx_opt)
        cv_file.write("data", mtx_opt)
        cv_file.release()

        return mtx_opt

if __name__ == '__main__':
    print("python")
    amcl_pose_addr = "/home/mjlee/ws/src/external_ros/ORB_SLAM2/amcl_pose_output.txt"
    orb_pose_addr = "/home/mjlee/ws/src/external_ros/ORB_SLAM2/orb_pose_output.txt"
    MapCalibration(amcl_pose_addr, orb_pose_addr)
