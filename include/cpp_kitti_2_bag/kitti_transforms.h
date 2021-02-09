
#ifndef CPP_KITTI_2_BAG_KITTI_TRANSFORMS_H_
#define CPP_KITTI_2_BAG_KITTI_TRANSFORMS_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <tf2_msgs/TFMessage.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

geometry_msgs::TransformStamped getStaticTransform(Eigen::MatrixXf t, Eigen::Quaternionf quaternion);
Eigen::MatrixXf calibVeloImu(std::string filename);
Eigen::Matrix4f inv(Eigen::Matrix3f R, Eigen::MatrixXf t, Eigen::Matrix4f &tfVelo);
#endif