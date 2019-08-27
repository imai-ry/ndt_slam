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
  
  int _method_type;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> _ndt;
  pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> _omp_ndt;
  float _scan_shift;
  
  
  Eigen::Affine3f _tf_btol, _tf_ltob;
  Eigen::Vector3f _diff_pose, _previous_pose;

  std::string _map_frame_id;

  bool _is_first_scan;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr _map_ptr;
  
  // NDT parameter
  float _voxel_leaf_size;
  float _trans_eps;
  float _step_size;
  float _ndt_res;
  int   _max_iter;
  
  void voxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &out,
                             float                                leaf_size);
  void calucurateInitGuess(Eigen::Matrix4f &init_guess);
  void ndt(const pcl::PointCloud<pcl::PointXYZI>::Ptr      &source,
           const pcl::PointCloud<pcl::PointXYZI>::Ptr      &target,
           const Eigen::Matrix4f                           &init_guess,
                 Eigen::Matrix4f                           &t_localizer);
  
public:
  NDT_SLAM();
  void setup(ros::NodeHandle nh, ros::NodeHandle private_nh);
  void start();
  
  
};

#endif 
