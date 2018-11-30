#ifndef INCLUDE_H
#define INCLUDE_H
#include <string>
using namespace std;

typedef struct config{
    double cam_fx, cam_fy, cam_cx, cam_cy, cam_k1, cam_k2, cam_p1, cam_p2, cam_k3;
    int cam_width, cam_height;
    double cam_fps, cam_bf;
    int cam_rgb;
    
    double depth_map_factor, th_depth;
    double orb_scale_factor;
    int orb_n_features , orb_n_levels, orb_ini_th, orb_min_th;
    float viewer_frame_size, viewer_graph_line_width, viewer_cam_size, viewer_pointX, viewer_pointY, viewer_pointZ;
    int viewer_frame_line_width, viewer_point_size, viewer_cam_line_width, viewer_pointF;
    string map_loadfile, map_savefile;
    string cam_vocab, cam_calib;
    int rsc_itr;
    double rsc_th, rsc_time_interval;
    string ros_cam_pose_topic, ros_robot_pose_topic, ros_rgb_topic, ros_depth_topic;
} CONFIG;

#endif // SYSTEM_H
