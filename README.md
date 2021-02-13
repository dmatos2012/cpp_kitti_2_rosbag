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
Go to your creted ROS package, and do

`catkin_make`

If built successfully, then just run the node with 

`rosrun cpp_kitti_2_bag main`

which will create the rosbag in your root folder and you are done.

## Acknowledgements
Heavily inspired by the existing tools to convert from raw dataset to KITTI bag, for [Python](https://github.com/tomas789/kitti2bag)  and for [C++](https://github.com/ethz-asl/kitti_to_rosbag) implementation
