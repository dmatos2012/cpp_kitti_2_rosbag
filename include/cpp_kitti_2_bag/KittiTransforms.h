#ifndef CPP_KITTI_2_BAG_KITTITRANSFORMS_H
#define CPP_KITTI_2_BAG_KITTITRANSFORMS_H


#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <experimental/filesystem>
#include <tf2_msgs/TFMessage.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include "cpp_kitti_2_bag/KittiTransforms.h"

namespace fs = std::experimental::filesystem;

class KittiTransforms
{
private:
    const std::string veloFrameId = "velo_link";
    const std::string imuFrameId = "imu_link";
    const std::string pointCloudTopic = "/kitti/velo";
    const std::string tfTopic = "/tf_static";
    const fs::path veloCalibFilename = "calib_imu_to_velo.txt";
    const fs::path timeStampsFilename = "timestamps.txt";
    
public:
    KittiTransforms();    
    geometry_msgs::TransformStamped getStaticTransform(Eigen::MatrixXf t, Eigen::Quaternionf quaternion);
    Eigen::MatrixXf extractTf(std::string filename);
    Eigen::Matrix4f inv(Eigen::Matrix3f R, Eigen::MatrixXf t, Eigen::Matrix4f &tfVelo);
    

};

#endif