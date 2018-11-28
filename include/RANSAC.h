
#ifndef RANSAC_H
#define RANSAC_H

namespace ORB_SLAM2
{

class RANSAC
{
public:
    RANSAC(std::vector<std::vector<float>>&, std::vector<std::vector<float>>&, int, float, float);
    void getMtx(Eigen::Matrix4f&);

private:
    std::vector<std::vector<float>> raw_orb_poses;
    std::vector<std::vector<float>> raw_amcl_poses;
    std::vector<std::vector<float>> orb_poses;
    std::vector<std::vector<float>> amcl_poses;
    Eigen::Matrix4f mtx_opt;
    void fitting();
    float error(Eigen::Matrix4f&, Eigen::Matrix4f&, Eigen::Matrix4f&);

    int rsc_itr_num;
    float rsc_thresh;
    float time_interval;
    int tf_compute_interval;

    void load_data(std::vector<std::vector<float>>&, std::vector<std::vector<float>>&);
    void vec2mat(std::vector<float>&, Eigen::Matrix4f&);
};
}

#endif //RANSAC_H