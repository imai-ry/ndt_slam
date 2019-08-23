#ifndef NDT_SLAM_H
#define NDT_SLAM_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class NDT_SLAM
{
private:
  void callback(const sensor_msgs::PointCloud2::ConstPtr& input);
  ros::NodeHandle _nh, _private_nh;
  ros::Subscriber _sub;
public:
  NDT_SLAM();
  void setup(ros::NodeHandle nh, ros::NodeHandle private_nh);
  void start();
};

#endif 
