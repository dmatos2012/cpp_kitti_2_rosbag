#include <iostream>
#include <Eigen/Dense>
#include <experimental/filesystem>
#include "cpp_kitti_2_bag/KittiTransforms.h"
#include "cpp_kitti_2_bag/KittiParser.h"


namespace fs = std::experimental::filesystem;
using namespace std;


    
    // KittiParser::KittiParser parser{rootPath, veloPointsPath};









int main()
{
    Eigen::MatrixXf veloCalib;
    const fs::path rootPath = fs::current_path();
    const fs::path dataPath = rootPath / "data/2011_09_26";
    const fs::path rawFile = "2011_09_26_drive_0011_sync";
    const fs::path veloPointsPath = dataPath / rawFile / "velodyne_points";
    KittiTransforms tfKitti{};
    veloCalib = tfKitti.extractTf(dataPath.string() + "/calib_imu_to_velo.txt");

    KittiParser parser{rootPath, veloPointsPath};
    // std::vector <std::string> veloFiles = extractVeloFiles(rootPath);
// std::vector <std::string> timestamps = extractTimestamps(timestampFile);


    // KittiParser parser(rootPath, veloPointsPath);
    return 0;

    // KittiTransforms::KittiTransforms tfKitti{};
    // KittiTransforms tfKitti{};
    // veloCalib = tfKitti.extractTf(dataPath.string() + "/calib_imu_to_velo.txt");
    // veloCalib = tfKitti.extractTf()::veloCalibFilename
    //path to calib.imuvelo
    // veloCalib =tfKitti.extractTf
    // // KittiParser(const fs::path rootPath, const fs::path veloPointsDir):
    // cout<<"hello";
    // KittiParser(fs::path calibrationFilename, fs::path dataPath);
    // KittiTransforms ktf;
    // Eigen::MatrixXf tfVelo;
    // tfVelo = ktf.extractTf(veloCalibFilename)

}