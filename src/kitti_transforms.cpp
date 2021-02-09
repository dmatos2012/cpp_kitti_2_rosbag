#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <tf2_msgs/TFMessage.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include "cpp_kitti_2_bag/kitti_transforms.h"



using namespace std;
using namespace Eigen;

struct veloTf
{
    vector <float> R;
    vector <float> T;
    
};

MatrixXf calibVeloImu(string filename)
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
    // MatrixXf tfVelo(RMat.rows(), RMat.cols()+tMat.cols());
    Matrix4f tfVelo(4,4);

    inv(RMat, tMat, tfVelo); //tMat
    // do I Need the [0,0,0,1] to make it square.probs. 

    // use inv here. 
    // tfVelo << RMat, tMat; // 3 x 4 matrix
    return tfVelo;

    

}

// void load_calib_rigid()
// {

// }


// Quaternion<double> getQuaternionFromRotationMatrix(const Matrix3d& mat)
// {
//     Matrix3f mat;
//     Quaternionf q(mat);
// }

geometry_msgs::TransformStamped getStaticTransform(MatrixXf t, Quaternionf quaternion)
{
    geometry_msgs::TransformStamped tf_msg;
    string velo_frame_id = "velo_link";
    string imu_frame_id = "imu_link";
    tf_msg.header.frame_id = imu_frame_id;
    tf_msg.child_frame_id = velo_frame_id;
    tf_msg.transform.translation.x = t(0);
    tf_msg.transform.translation.y = t(1);
    tf_msg.transform.translation.z = t(2);
    tf_msg.transform.rotation.x = quaternion.x();
    tf_msg.transform.rotation.y = quaternion.y();
    tf_msg.transform.rotation.z = quaternion.z();
    tf_msg.transform.rotation.w = quaternion.w();
    return tf_msg;

}

Matrix4f inv(Matrix3f R, MatrixXf t, Matrix4f &tfVelo)
{
    MatrixXf t_inv(3,1);
    Matrix3f R_inv(3,3);
    MatrixXf fillMat(1,4);
    t_inv << -1 * R.transpose() * t; // not dot as is for vectors
    R_inv << R.transpose();
    fillMat << 0,0,0,1;
    tfVelo << R_inv, t_inv, fillMat;
    cout << "tf velo"<<tfVelo;

    return tfVelo;
    // Invert rigid body transformation matrix

}
int main() 
{
    MatrixXf tfVelo;
    string filename = "./data/2011_09_26/calib_imu_to_velo.txt";
    tfVelo = calibVeloImu(filename);
    
    
    
    // tf_msg.header.frame_id
    // tf_msg.child_frame_id
  
    // m.getRotation()
    // tf2::Transform 

    // https://answers.ros.org/question/289144/apply-transformation-matrix-to-existing-frame/


}

