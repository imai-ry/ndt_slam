#ifndef NDT_SLAM_H
#define NDT_SLAM_H
//#define USE_OPENMP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#ifdef USE_OPENMP
#include <pcl_omp_registration/ndt.h>
#endif

#include <eigen3/Eigen/Geometry>

class NDT_SLAM
{
private:

  typedef struct
  {
    double x; double y; double z; double roll; double pitch; double yaw;
  } Pose;
  
  Pose _diff_pose, _previous_pose, _added_pose;

  // publisher & subscriber
  ros::Subscriber _sub;
  ros::Publisher _map_pub;
  
  // transform between base and lidar
  Eigen::Matrix4f _tf_btol, _tf_ltob;
  
  //
  int _method_type;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> _ndt;
#ifdef USE_OPENMP
  pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> _omp_ndt;
#endif
  float _scan_shift;

  std::string _map_frame_id;

  bool _is_first_scan, _is_first_map;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr _map_ptr;
  
  // NDT parameter
  double _voxel_leaf_size;
  double _trans_eps;
  double _step_size;
  float _ndt_res;
  int   _max_iter;
  
  void callback(const sensor_msgs::PointCloud2::ConstPtr& input);
  void voxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &out,
                             double                               voxel_size);
  /*
  void calucurateInitGuess(Eigen::Matrix4f &init_guess);
  */
  void ndt(const pcl::PointCloud<pcl::PointXYZI>::Ptr      &source,
           const pcl::PointCloud<pcl::PointXYZI>::Ptr      &target,
            Eigen::Matrix4f                           &init_guess,
            Eigen::Matrix4f                           &t_localizer);
            
  double calcDiffForRadian(const double lhs_rad, const double rhs_rad);

  
public:
  NDT_SLAM(ros::NodeHandle nh, ros::NodeHandle private_nh);
 
};

#endif 
