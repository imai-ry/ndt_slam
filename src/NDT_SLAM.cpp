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
  
  _initial_scan = true;
  _is_first_map = true;
  _map_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  
  if(!(_private_nh.getParam("map_frame_id", _map_frame_id))) 
    ROS_BREAK();
  
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
  if(!(_private_nh.getParam("voxel_leaf_size", _voxel_leaf_size)) ||
     !(_private_nh.getParam("trans_eps", _trans_eps)) ||
     !(_private_nh.getParam("step_size", _ndt_res)) ||
     !(_private_nh.getParam("ndt_res", _ndt_res)) ||
     !(_private_nh.getParam("max_iter", _max_iter)))
    ROS_BREAK();
  
  _ndt.setTransformationEpsilon(_trans_eps);
  _ndt.setStepSize(_step_size);
  _ndt.setResolution(_ndt_res);
  _ndt.setMaximumIterations(_max_iter);
}

void NDT_SLAM::start()
{
  // register callback
  _sub = _nh.subscribe("pointcloud", 1000, &NDT_SLAM::callback, this);
}

void NDT_SLAM::callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{ 
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::fromROSMsg(*input, scan);
  
  // limit distanace
  
  // from lidar coordinate to base coordinate
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(scan, *transformed_scan_ptr, _tf_btol);
  
  // add initial point cloud to map
  if(_initial_scan == true)
  {
    *_map_ptr += *transformed_scan_ptr;
    _initial_scan = false;
    //return;
  }
  
  // Apply voxelgrid filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(_voxel_leaf_size, _voxel_leaf_size, _voxel_leaf_size);
  voxel_grid_filter.setInputCloud(transformed_scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr); 
  
  // caluculate init guess
  Eigen::Translation3f t(0,0,0); //kari
  Eigen::Affine3f at;
  at = t;
  Eigen::Matrix4f init_guess = at.matrix();
 
  // NDT matching  map <=> filterd_scan 
  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  if(_is_first_map == true)
  {
    _ndt.setInputTarget(_map_ptr);
    _is_first_map = false;
  }
  _ndt.setInputSource(filtered_scan_ptr);
  
  _ndt.align(output_cloud, init_guess);
  //fitness_score = ndt.getFitnessScore();
  Eigen::Matrix4f pose_change = _ndt.getFinalTransformation();
  //has_converged = ndt.hasConverged();
  //final_num_iteration = ndt.getFinalNumIteration();
  //transformation_probability = ndt.getTransformationProbability();
  ROS_INFO("NDT");
  

  // add scan to map 
  pcl::transformPointCloud(*transformed_scan_ptr, output_cloud, pose_change);
  *_map_ptr += output_cloud;
  
  // publish map
  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(*_map_ptr, map_msg);
  map_msg.header.frame_id = _map_frame_id;
  _map_pub.publish(map_msg);
}


