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

#include <pcl_omp_registration/ndt.h>

#include <eigen3/Eigen/Geometry>

class NDT_SLAM
{
private:
  void callback(const sensor_msgs::PointCloud2::ConstPtr& input);
  ros::NodeHandle _nh, _private_nh;
  ros::Subscriber _sub;
  ros::Publisher _map_pub;
  Eigen::Affine3f _tf_btol;
  Eigen::Matrix4f _global_pose_change, _previous_pose_change;

  std::string _map_frame_id;

  bool _is_first_scan;
  bool _is_first_map;
  
  int _method_type;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> _ndt;
  pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> _omp_ndt;
  float _scan_shift;
  
  pcl::PointCloud<pcl::PointXYZI>      _map;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _filtered_previous_cloud_ptr;
  
  // NDT parameter
  float _voxel_leaf_size;
  float _trans_eps;
  float _step_size;
  float _ndt_res;
  int   _max_iter;
  
  void voxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &out);
  void calucurateInitGuessPoseChange(Eigen::Matrix4f &init_guess_pose_change);
  void ndt(const pcl::PointCloud<pcl::PointXYZI>::Ptr      &source,
           const pcl::PointCloud<pcl::PointXYZI>::Ptr      &target,
           const Eigen::Matrix4f                           &init_guess_pose_change,
                 Eigen::Matrix4f                           &pose_change);
  
public:
  NDT_SLAM();
  void setup(ros::NodeHandle nh, ros::NodeHandle private_nh);
  void start();
  
  
};

#endif 
