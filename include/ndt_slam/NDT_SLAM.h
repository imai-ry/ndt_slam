#ifndef NDT_SLAM_H
#define NDT_SLAM_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#include <eigen3/Eigen/Geometry>

class NDT_SLAM
{
private:
  void callback(const sensor_msgs::PointCloud2::ConstPtr& input);
  ros::NodeHandle _nh, _private_nh;
  ros::Subscriber _sub;
  ros::Publisher _map_pub;
  Eigen::Affine3f _tf_btol;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _map_ptr;
  bool _initial_scan;
  bool _is_first_map;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> _ndt;
  
  // NDT parameter
  float _voxel_leaf_size;
  float _trans_eps;
  float _step_size;
  float _ndt_res;
  int   _max_iter;
  
public:
  NDT_SLAM();
  void setup(ros::NodeHandle nh, ros::NodeHandle private_nh);
  void start();
};

#endif 
