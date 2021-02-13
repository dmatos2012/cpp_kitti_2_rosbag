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
    const std::string pointCloudTopic = "/kitti/velo";

    KittiParser(const fs::path rootPath, const fs::path timeStampsFilename);

    int64_t parseStamp(std::string timestamp);
    ros::Time timestampToRos(uint64_t timestamp_ns, ros::Time* time);
    bool getPointCloud(std::string veloFile, pcl::PointCloud<pcl::PointXYZI>* ptcloud);
    std::vector <std::string> extractTimestamps();
    std::vector <std::string> extractVeloFiles();

private:
    fs::path m_veloPointsPath;
    fs::path m_timeStampsFile;

};
#endif