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

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Geometry>

class NDT_SLAM
{
private:

  typedef struct
  {
    double x; double y; double z; double roll; double pitch; double yaw;
  } Pose;
  
  Pose _diff_pose, _current_pose, _previous_pose, _added_pose, _guess_pose, _ndt_pose;

  // publisher & subscriber
  ros::Subscriber _sub;
  ros::Publisher _map_pub;
  
  // transform between base and lidar
  Eigen::Matrix4f _tf_btol, _tf_ltob;
  
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> _ndt;
  pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> _omp_ndt;

  
  //
  int _method_type;
  float _scan_shift;

  std::string _map_frame_id;

  bool _is_first_scan, _is_first_map;
  pcl::PointCloud<pcl::PointXYZI> _map;
 
  
  // NDT parameter
  double _voxel_leaf_size;
  double _trans_eps;
  double _step_size;
  float _ndt_res;
  int   _max_iter;
  
  void callback(const sensor_msgs::PointCloud2::ConstPtr& input);          
  double calcDiffForRadian(const double lhs_rad, const double rhs_rad);

  
public:
  NDT_SLAM(ros::NodeHandle nh, ros::NodeHandle private_nh);
 
};

#endif 
