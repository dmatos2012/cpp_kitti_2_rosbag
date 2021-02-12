#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <experimental/filesystem>
#include "cpp_kitti_2_bag/KittiTransforms.h"
#include "cpp_kitti_2_bag/KittiParser.h"
#include <boost/range/combine.hpp>
#include <rosbag/bag.h>
#include <pcl_ros/point_cloud.h>

namespace fs = std::experimental::filesystem;
using namespace std;


    
    // KittiParser::KittiParser parser{rootPath, veloPointsPath};



void writeVeloBag(rosbag::Bag *bag_, KittiParser &parser, string veloFile, string timeStamp, string outBag)
{
     pcl::PointCloud<pcl::PointXYZI> pointcloud;
    //  rosbag::Bag bag_;
    //  bag_.open("./data/" + outBag, rosbag::bagmode::Write);
     ros::Time timeStampRos;

    

    if (parser.getPointCloud(veloFile, &pointcloud))
    {
        uint64_t parsedStamp = parser.parseStamp(timeStamp); 
        parser.timestampToRos(parsedStamp, &timeStampRos);
        pointcloud.header.stamp = parsedStamp/1000; //since value in microseconds.
        pointcloud.header.frame_id = parser.veloFrameId;
        // bag_.write(parser.pointCloudTopic, timeStampRos, pointcloud);
        bag_->write(parser.pointCloudTopic, timeStampRos, pointcloud);


    }
    // bag_.close();
    
}

void writeTf(rosbag::Bag *bag_, geometry_msgs::TransformStamped tf_msg, ros::Time timeStampRos, tf2_msgs::TFMessage tfm)
{

    tf_msg.header.stamp = timeStampRos;
    tfm.transforms.push_back(tf_msg);
    bag_->write("/tf_static", timeStampRos, tfm);
}

int main()
{
    rosbag::Bag bag_;
    bag_.open("test1.bag", rosbag::bagmode::Write);
    tf2_msgs::TFMessage tfm;
    

    const fs::path rootPath = fs::current_path();
    const fs::path dataPath = rootPath / "data/2011_09_26";
    const fs::path rawFile = "2011_09_26_drive_0011_sync";
    const string bagOutName = rawFile.string()+ ".bag";
    const fs::path veloPointsPath = dataPath / rawFile / "velodyne_points";
    const fs::path timeStampsFilename = veloPointsPath / "timestamps.txt";
    KittiTransforms tfKitti{};
    Eigen::MatrixXf veloCalib = tfKitti.extractTf(dataPath.string() + "/calib_imu_to_velo.txt");
    Eigen::Quaternionf quaternion(veloCalib.topLeftCorner<3,3>()); //for rotations
    Eigen::MatrixXf t = veloCalib.topRightCorner<3,1>(); //translations

    KittiParser parser{veloPointsPath,  timeStampsFilename};
    vector <string> veloFiles = parser.extractVeloFiles();
    vector <string> timestamps = parser.extractTimestamps();
    geometry_msgs::TransformStamped tf_msg = tfKitti.getStaticTransform(t, quaternion);
    ros::Time timeStampRos;

    


    for (auto const& i : boost::combine(veloFiles, timestamps))
    {
        string veloFile;
        string timeStamp;
        boost::tie(veloFile,timeStamp) = i;
        // writeVeloBag(parser, veloFile, timeStamp, bagOutName); //can i make it const?
        writeVeloBag(&bag_, parser, veloFile, timeStamp, bagOutName); //can i make it const?

        uint64_t parsedStamp = parser.parseStamp(timeStamp); 
        parser.timestampToRos(parsedStamp, &timeStampRos);
        writeTf(&bag_, tf_msg, timeStampRos, tfm);
    }
    bag_.close();
    return 0;
}
