#include <ndt_slam/NDT_SLAM.h>

NDT_SLAM::NDT_SLAM()
{
}

void NDT_SLAM::setup(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  using namespace Eigen;

  _nh = nh;
  _private_nh = private_nh;
  _map_pub = _nh.advertise<sensor_msgs::PointCloud2>("ndt_map", 1);
  
  _initial_scan = false;
  
  // get transform base(global) coordinate to lidar coordinate
  float x,y,z,roll,pitch,yaw;
  if(!(_private_nh.getParam("tf_btol_x", x)) ||
     !(_private_nh.getParam("tf_btol_y", y)) ||
     !(_private_nh.getParam("tf_btol_z", z)) ||
     !(_private_nh.getParam("tf_btol_roll", roll)) ||
     !(_private_nh.getParam("tf_btol_pitch", pitch)) ||
     !(_private_nh.getParam("tf_btol_yaw", yaw)) )
    ROS_BREAK();
  
  _tf_btol = AngleAxisf(yaw, Vector3f::UnitZ())
             *AngleAxisf(pitch, Vector3f::UnitY())
             *AngleAxisf(roll, Vector3f::UnitX())
             *Translation3f(x,y,z);
             
  // get NDT parameter
  if(!(_private_nh.getParam("voxel_leaf_size", _voxel_leaf_size)))
    ROS_BREAK();
  
}

void NDT_SLAM::start()
{
  // register callback
  _sub = _nh.subscribe("pointcloud", 1, &NDT_SLAM::callback, this);
}

void NDT_SLAM::callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{ 
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::fromROSMsg(*input, scan);
  
  // limit distanace
  
  // from lidar coordinate to global coordinate
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(scan, *transformed_scan_ptr, _tf_btol);
  
  // add initial point cloud to map
  if(_initial_scan == false)
  {
    _map += *transformed_scan_ptr;
    _initial_scan = true;
  }
  
  // Apply voxelgrid filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(_voxel_leaf_size, _voxel_leaf_size, _voxel_leaf_size);
  voxel_grid_filter.setInputCloud(transformed_scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr); 
 
  // NDT matching  map <=> filterd_scan 
  
  // publish map
  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(_map, map_msg);
  map_msg.header.frame_id = "map"; // "velodyne"
  _map_pub.publish(map_msg);
}


