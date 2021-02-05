#include <string>
// #include "std_msgs/Header.h"
#include <iostream>
#include <vector>
#include <experimental/filesystem>
#include <sstream>
#include <fstream>
#include <rosbag/bag.h>
// #include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <time.h>
#include <cpp_kitti_2_bag/date.h>

namespace fs = std::experimental::filesystem;

std::vector <std::string> extractVeloFiles(std::string rootPath)
{
    std::vector <std::string> veloFiles;

    for (const auto & entry : fs::directory_iterator(rootPath + "/data"))
        veloFiles.push_back(entry.path());
    return veloFiles;
        // std::cout << entry.path() << std::endl;


}

std::vector <std::string> extractTimestamps(std::string rootPath)
{
    std::vector <std::string> timestamps;
    std::string date, time;
    std::string line;
    std::ifstream file(rootPath);
    
    while (getline(file, line))
    {
        std::istringstream sline(line);
        while(sline>>date>>time)
        {
            timestamps.push_back(time);
            // std::cout<<date<<time<<std::endl;
        }

    }
    return timestamps;

    // for (const auto & entry : fs::directory_iterator(rootPath))
    //     timestamps.push_back(entry.path());
    // return timestamps;
    //     // std::cout << entry.path() << std::endl;


}

bool getPointCloud(std::string veloFile, pcl::PointCloud<pcl::PointXYZI>* ptcloud)
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

ros::Time timestampToRos(uint64_t timestamp_ns, ros::Time* time)
{
    time->fromNSec(timestamp_ns);

}

uint64_t touint64(std::string time_string) 
{
    uint64_t time_ros;
    std::istringstream iss(time_string);
    iss >> time_ros;
    return time_ros;

}


int main()
{
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::system_clock;
    std::chrono::system_clock::time_point tp;
    std::istringstream ss{"2011-09-26 13:02:44.317412775"};
    ss >> date::parse("%F %T", tp);
    using date::operator<<;
    std::cout << "Date/Time is " << tp << std::endl;
    auto millisec_since_epoch = duration_cast<milliseconds>(tp.time_since_epoch()).count();
    std::cout<<millisec_since_epoch;
    struct tm timeDate;
    rosbag::Bag bag_;
    bag_.open("./src/cpp_kitti_2_bag/data/test2.bag", rosbag::bagmode::Write);
    std::string velo_frame_id = "velo_link";
    std::string pointcloud_topic = "/kitti/velo";
    uint64_t ts;
    char time_buffer[100];
    ros::Time timestamp_ros;
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    std::string rootPath = "/home/david/catkin_ws/src/cpp_kitti_2_bag/data/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points";
    std::string timestampFile = rootPath + "/timestamps.txt";
    std::vector <std::string> veloFiles = extractVeloFiles(rootPath);
    std::vector <std::string> timestamps = extractTimestamps(timestampFile);

    for (int i=0; i<4;i++)
    {
        if (getPointCloud(veloFiles[i], &pointcloud))
               
            std::cout<<"successful"<<std::endl;
         
        else
        {
            std::cout<<"Error opening file";
        }
        strptime(timestamps[i].c_str(), "%H:%M:%S.%f", &timeDate);
        strftime(time_buffer, 50, "%s", &timeDate);
        // std::cout<<"timedate"<<timeDate<<std::endl;
    
        ts = touint64(timestamps[i]);
        timestampToRos(ts, &timestamp_ros);
        pointcloud.header.stamp = ts;
        pointcloud.header.frame_id = velo_frame_id;
        bag_.write(pointcloud_topic, timestamp_ros, pointcloud);
        
    }
    bag_.close();
    
    // uint64_t time = pcl_conversions::toPCL(ros::Time::now(), pointcloud.header.stamp)
    // pointcloud.header.stamp = pcl_conversions::toPCL(ros::Time::now(), pointcloud.header.stamp);

}

