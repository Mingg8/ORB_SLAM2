#ifndef INCLUDE_H
#define INCLUDE_H

using namespace std;

typedef struct config{
    std::string cam_fx, cam_fy, cam_cx, cam_cy, cam_k1, cam_k2, cam_p1, cam_p2, cam_k3, cam_width, cam_height, cam_fps, cam_bf, cam_rgb;
    std::string depth_map_factor, th_depth;
    std::string orb_scale_factor, orb_n_features , orb_n_levels, orb_ini_th, orb_min_th;
    std::string viewer_frame_size, viewer_graph_line_width, viewer_cam_size, viewer_pointX, viewer_pointY, viewer_pointZ, viewer_frame_line_width, viewer_point_size, viewer_cam_line_width, viewer_pointF;
    std::string map_loadfile, map_savefile;
    std::string cam_vocab, cam_calib;
    std::string rsc_itr, rsc_th, rsc_time_interval;
    std::string ros_cam_pose_topic, ros_robot_pose_topic, ros_rgb_topic, ros_depth_topic;
} CONFIG;

#endif // SYSTEM_H
