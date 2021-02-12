#include <string>
#include <iostream>
#include <vector>
#include <experimental/filesystem>
#include <sstream>
#include <fstream>
#include <rosbag/bag.h>
#include <pcl_ros/point_cloud.h>
#include <time.h>
#include <cpp_kitti_2_bag/date.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tfMessage.h>
// #include "cpp_kitti_2_bag/KittiTransforms.h"
#include "cpp_kitti_2_bag/KittiParser.h"
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;
// namespace fs = std::experimental::filesystem;




KittiParser::KittiParser(const fs::path veloPoints, const fs::path timestampsFilename)
: m_veloPointsPath {veloPoints}, m_timeStampsFile{timestampsFilename}
{
}



std::vector <std::string> KittiParser::extractVeloFiles()
{
    std::vector <std::string> veloFiles;

    for (const auto & entry : fs::directory_iterator(m_veloPointsPath.string() + "/data"))
        veloFiles.push_back(entry.path());

    std::sort(veloFiles.begin(), veloFiles.end(),
            [](const auto& lhs, const auto& rhs) {
                return lhs < rhs;
            });
    return veloFiles;
        // std::cout << entry.path() << std::endl;


}

std::vector <std::string> KittiParser::extractTimestamps()
{
    std::vector <std::string> timestamps;
    std::string date, time;
    std::string line;
    std::ifstream file(m_timeStampsFile.string());
    
    while (getline(file, line))
    {
        timestamps.push_back(line);
    }
    return timestamps;

    // for (const auto & entry : fs::directory_iterator(rootPath))
    //     timestamps.push_back(entry.path());
    // return timestamps;
    //     // std::cout << entry.path() << std::endl;


}

bool KittiParser::getPointCloud(std::string veloFile, pcl::PointCloud<pcl::PointXYZI>* ptcloud)
{
    // Load the actual pointcloud.
    const size_t kMaxNumberOfPoints = 1e6;  // From Readme for raw files.
    // only reading one as sample whether its getting items correctly. 
    ptcloud->clear();
    ptcloud->reserve(kMaxNumberOfPoints);


    std::ifstream inputVelo(veloFile, std::ios::in | std::ios::binary);
    if (!inputVelo)
    {
        std::cout<<"Could not open file";
        return false;
    }
    for (int i = 0; inputVelo.good() && !inputVelo.eof();i++)
    {
        pcl::PointXYZI point;
        inputVelo.read((char*)&point.x, 3 * sizeof(float));
        inputVelo.read((char*)&point.intensity,sizeof(float));
        ptcloud->push_back(point);
    }
    inputVelo.close();
    return true;

}


ros::Time KittiParser::timestampToRos(uint64_t timestamp_ns, ros::Time* time)
{
    time->fromNSec(timestamp_ns);


}

int64_t KittiParser::parseStamp(std::string timestamp)
{
    system_clock::time_point tp;
    std::istringstream ss(timestamp);
    ss >> date::parse("%F %T", tp);
    using date::operator<<;
    // std::cout << "Date/Time is " << tp << std::endl;
    int64_t nanosec_since_epoch = duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
    // auto nanosec_since_epoch = duration_cast<nano>(tp.time_since_epoch()).count();
    return nanosec_since_epoch; 

}
