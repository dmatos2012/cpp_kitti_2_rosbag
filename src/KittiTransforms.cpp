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

using namespace std;
using namespace Eigen;
namespace fs = std::experimental::filesystem;
KittiTransforms::KittiTransforms() = default; //constructor 

MatrixXf KittiTransforms::extractTf(string filename)
{
    string line;
    ifstream input(filename);
    vector <float> tokens;

    // Skip first line
    getline(input, line);
    while(getline(input, line))
    {
        istringstream ss(line);
        while(getline(ss,line, ' '))
        {
            if (line != "R:" && line !="T:")
                tokens.push_back(stof(line));

        }

    }
    vector <float> R{tokens.begin(),tokens.begin()+ 9};
    Matrix3f RMat = Map<Matrix<float,3,3>>(R.data()).transpose(); //to make it like np.reshape
    vector <float> t{tokens.begin()+9,tokens.end()};
    MatrixXf tMat = Map<Matrix<float,3,1>>(t.data());
    Matrix4f tfVelo(4,4);

    KittiTransforms::inv(RMat, tMat, tfVelo); //tMat
    return tfVelo;
}

Matrix4f KittiTransforms::inv(Matrix3f R, MatrixXf t, Matrix4f &tfVelo)
{
    MatrixXf t_inv(3,1);
    Matrix3f R_inv(3,3);
    MatrixXf fillMat(1,4);
    t_inv << -1 * R.transpose() * t; // not dot as is for vectors
    R_inv << R.transpose();
    fillMat << 0,0,0,1;
    tfVelo << R_inv, t_inv, fillMat;
    return tfVelo;
}

geometry_msgs::TransformStamped KittiTransforms::getStaticTransform(MatrixXf t, Quaternionf quaternion)
{
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.frame_id = "imu_link";
    tf_msg.child_frame_id = "velo_link";
    tf_msg.transform.translation.x = t(0);
    tf_msg.transform.translation.y = t(1);
    tf_msg.transform.translation.z = t(2);
    tf_msg.transform.rotation.x = quaternion.x();
    tf_msg.transform.rotation.y = quaternion.y();
    tf_msg.transform.rotation.z = quaternion.z();
    tf_msg.transform.rotation.w = quaternion.w();
    return tf_msg;

}
        // Invert rigid body transformation matrix




    



// void load_calib_rigid()
// {

// }


// Quaternion<double> getQuaternionFromRotationMatrix(const Matrix3d& mat)
// {
//     Matrix3f mat;
//     Quaternionf q(mat);
// }

