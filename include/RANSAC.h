
#ifndef RANSAC_H
#define RANSAC_H

namespace ORB_SLAM2
{

class RANSAC
{
public:
    RANSAC(std::vector<std::vector<float>>, std::vector<std::vector<float>>);
    Eigen::Matrix4f* getMtx();

private:
    std::vector<std::vector<float>> orb_poses;
    std::vector<std::vector<float>> amcl_poses;
    Eigen::Matrix4f mtx_opt;
    void fitting();
    float error(Eigen::Matrix4f*, Eigen::Matrix4f*, Eigen::Matrix4f*);
    float error(std::vector<float>*, std::vector<float>*);

    int itr_num;
    int threshold;
    int n_data;
};
}

#endif //RANSAC_H