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
  ros::Publisher _map_pub, _map_pub2;
  Eigen::Affine3f _tf_btol;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _map_ptr;
  std::string _map_frame_id;

  bool _initial_scan;
  bool _is_first_map;
  
  int _method_type;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> _ndt;
  pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> _omp_ndt;
  float _scan_shift;
  
  // NDT parameter
  float _voxel_leaf_size;
  float _trans_eps;
  float _step_size;
  float _ndt_res;
  int   _max_iter;
  
  // pose
  typedef struct
  {
    float x;
    float y;
    float z;
  } Pose;
  Pose _current_pose, _estimated_pose;
  
public:
  NDT_SLAM();
  void setup(ros::NodeHandle nh, ros::NodeHandle private_nh);
  void start();
};

#endif 
