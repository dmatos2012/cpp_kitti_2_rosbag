#ifndef CPP_KITTI_2_BAG_KITTIPARSER_H
#define CPP_KITTI_2_BAG_KITTIPARSER_H



#include <string>
#include <vector>
#include <time.h>
#include <experimental/filesystem>
#include <pcl_ros/point_cloud.h>
namespace fs = std::experimental::filesystem;
class KittiParser
{
public:
    const std::string veloFrameId = "velo_link";
    const std::string imuFrameId = "imu_link";
    const std::string pointCloudTopic = "/kitti/velo";
    const std::string tfTopic = "/tf_static";
    const fs::path m_rootPath = fs::current_path().root_path();
    const fs::path m_dataPath = "/data/2011_09_26/2011_09_26_drive_0011_sync";
    const fs::path m_veloPointsDir = "velodyne_points";

    KittiParser(const fs::path rootPath, const fs::path veloPointsDir);
    
    int64_t stampToTime(std::string timestamp);
    ros::Time timestampToRos(uint64_t timestamp_ns, ros::Time* time);
    bool getPointCloud(std::string veloFile, pcl::PointCloud<pcl::PointXYZI>* ptcloud);
    std::vector <std::string> extractTimestamps(std::string rootPath);
    std::vector <std::string> extractVeloFiles(std::string rootPath);

    

//double check class.h constructor 
    // KittiParser(fs::path calibrationFilename, fs::path dataPath);
    
    // const fs::path veloCalibFilename = "calib_imu_to_velo.txt";
    // const fs::path timeStampsFilename = "timestamps.txt";
    
    
// private:


};
#endif