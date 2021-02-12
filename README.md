## CPP_KITTI_2_BAG
Convert a KITTI raw dataset to ROS bag! 
Make sure you create a a catkin workspace properly.

## Introduction
A C++ project that I have used to further expand my C++ knowledge, and apply what I have learned in tutorials. With this tool, you can convert any raw dataset into a ROS bag. 

## TODOs
* Allow user input
* Add image messages
* Add unit tests.


## Usage
`rosrun cpp_kitti_2_bag main`

which will create the rosbag in your root folder.

## Acknowledgements
Heavily inspired by the existing tools to convert from raw dataset to KITTI bag, for [Python](https://github.com/tomas789/kitti2bag)  and for [C++](https://github.com/ethz-asl/kitti_to_rosbag) implementation
