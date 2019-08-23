#include <ndt_slam/NDT_SLAM.h>

NDT_SLAM::NDT_SLAM()
{
}

void NDT_SLAM::setup(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  _nh = nh;
  _private_nh = private_nh;
}

void NDT_SLAM::start()
{
  // register callback
  _sub = _nh.subscribe("pointcloud", 1, &NDT_SLAM::callback, this);
}

void NDT_SLAM::callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  ROS_INFO("get");
  // limit distanace
 
  
  // 
  
}


