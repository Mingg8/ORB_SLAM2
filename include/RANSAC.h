
#ifndef RANSAC_H
#define RANSAC_H

namespace ORB_SLAM2
{

class RANSAC
{
public:
    RANSAC(std::vector<std::vector<float>>&, std::vector<std::vector<float>>&);
    Eigen::Matrix4f* getMtx();

private:
    std::vector<std::vector<float>> raw_orb_poses;
    std::vector<std::vector<float>> raw_amcl_poses;
    std::vector<std::vector<float>> orb_poses;
    std::vector<std::vector<float>> amcl_poses;
    Eigen::Matrix4f mtx_opt;
    void fitting();
    float error(Eigen::Matrix4f&, Eigen::Matrix4f&, Eigen::Matrix4f&);

    int itr_num;
    int threshold;
    float time_interval;
    int tf_compute_interval;

    void load_data(std::vector<std::vector<float>>&, std::vector<std::vector<float>>&);
};
}

#endif //RANSAC_H