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
  
  _is_first_scan = true;
  
  _diff_pose = Eigen::Vector3f::Zero();
  _map_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  
  if(!(_private_nh.getParam("map_frame_id", _map_frame_id)) ||
     !(_private_nh.getParam("scan_shift", _scan_shift))) 
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
  
  _tf_ltob = _tf_btol.inverse();
             
  // get NDT parameter
  if(!(_private_nh.getParam("method_type", _method_type)) ||
     !(_private_nh.getParam("voxel_leaf_size", _voxel_leaf_size)) ||
     !(_private_nh.getParam("trans_eps", _trans_eps)) ||
     !(_private_nh.getParam("step_size", _ndt_res)) ||
     !(_private_nh.getParam("ndt_res", _ndt_res)) ||
     !(_private_nh.getParam("max_iter", _max_iter)))
    ROS_BREAK();
  
  if(_method_type == 0)
  {
    _ndt.setTransformationEpsilon(_trans_eps);
    _ndt.setStepSize(_step_size);
    _ndt.setResolution(_ndt_res);
    _ndt.setMaximumIterations(_max_iter);
  }
  else
  {
    _omp_ndt.setTransformationEpsilon(_trans_eps);
    _omp_ndt.setStepSize(_step_size);
    _omp_ndt.setResolution(_ndt_res);
    _omp_ndt.setMaximumIterations(_max_iter);
  }
  
}

void NDT_SLAM::start()
{
  // register callback
  _sub = _nh.subscribe("pointcloud", 1000, &NDT_SLAM::callback, this);
}


void NDT_SLAM::callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_lidar_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>      input_cloud_global;
  pcl::fromROSMsg(*input, *input_cloud_lidar_ptr);

  // if this is first map, register it to map directly
  if(_is_first_scan == true)
  {
    pcl::transformPointCloud(*input_cloud_lidar_ptr, input_cloud_global, _tf_btol);
    *_map_ptr += input_cloud_global;
    _is_first_scan = false;
  }
  
  // voxel grid filter on "input_cloud_lidar"
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_input_cloud_lidar_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  voxelGridFilter(input_cloud_lidar_ptr, filtered_input_cloud_lidar_ptr, _voxel_leaf_size);
  
  // calucurate NDT
  Eigen::Matrix4f init_guess, t_localizer;
  calucurateInitGuess(init_guess);
  ndt(filtered_input_cloud_lidar_ptr, _map_ptr, init_guess, t_localizer);
  
  // set pose
  Eigen::Matrix4f t_base_link;
  Eigen::Vector3f current_pose;
  t_base_link = t_localizer * _tf_ltob.matrix();
  current_pose(0) = t_base_link(0,3);  // x
  current_pose(1) = t_base_link(1,3);  // y
  current_pose(2) = t_base_link(2,3);  // z
  _diff_pose = current_pose - _previous_pose;
  _previous_pose = current_pose;
  
  // register to a map & publish map
  pcl::transformPointCloud(*input_cloud_lidar_ptr, input_cloud_global, t_localizer);
  *_map_ptr += input_cloud_global;
  sensor_msgs::PointCloud2 map;
  pcl::toROSMsg(*_map_ptr, map);
  map.header.frame_id = _map_frame_id;
  _map_pub.publish(map);

}

void NDT_SLAM::voxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &out,
                                     float                                leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.setInputCloud(in);
  filter.filter(*out); 
}

void NDT_SLAM::calucurateInitGuess(Eigen::Matrix4f &init_guess)
{
  Eigen::Vector3f guess_pose;
  guess_pose = _diff_pose;
  Eigen::Translation3f t(guess_pose(0), guess_pose(1), guess_pose(2));
  Eigen::Affine3f at;
  at = t * _tf_btol;
  init_guess = at.matrix();
}

void NDT_SLAM::ndt(const pcl::PointCloud<pcl::PointXYZI>::Ptr      &source,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr      &target,
                   const Eigen::Matrix4f                           &init_guess,
                         Eigen::Matrix4f                           &t_localizer)
{
  _omp_ndt.setInputTarget(target);
  
  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  if(_method_type==0)
  {
    _ndt.setInputSource(source);
    _ndt.align(output_cloud, init_guess);
    //fitness_score = ndt.getFitnessScore();
    t_localizer = _ndt.getFinalTransformation();
    //has_converged = ndt.hasConverged();
    //final_num_iteration = ndt.getFinalNumIteration();
    //transformation_probability = ndt.getTransformationProbability();
  }
  else
  {
    _omp_ndt.setInputSource(source);
    _omp_ndt.align(output_cloud, init_guess);
    //fitness_score = ndt.getFitnessScore();
    t_localizer = _omp_ndt.getFinalTransformation();
    //has_converged = _omp_ndt.hasConverged();
    //final_num_iteration = _omp_ndt.getFinalNumIteration();
    //transformation_probability = ndt.getTransformationProbability();
  } 
}


/*
void NDT_SLAM::callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI> input_cloud_lidar;
  pcl::fromROSMsg(*input, input_cloud_lidar);
  
  // limit distanace
  
  // from lidar coordinate to base coordinate
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_base_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(input_cloud_lidar, *input_cloud_base_ptr, _tf_btol);
  
  // on base coordinate, matching between last getted pcd and now getted pcd.
  // if this is first map, register it to map directly
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_global_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  Eigen::Matrix4f init_guess_pose_change, pose_change;
  pose_change = Eigen::Matrix4f::Identity();
  if(_is_first_scan == true)
  {
    input_cloud_global_ptr = input_cloud_base_ptr;
    _is_first_scan = false;
  }
  else
  {
    // ndt matching
    calucurateInitGuessPoseChange(init_guess_pose_change);
    ndt(_filtered_previous_cloud_ptr, input_cloud_base_ptr, init_guess_pose_change, pose_change);
    
    // from base coordinate to global coordinate
    _global_pose_change = pose_change * _global_pose_change;
    pcl::transformPointCloud(*input_cloud_base_ptr, *input_cloud_global_ptr, _global_pose_change);
  }
  
  // filter cloud to prevent "out of memory", and register input cloud to a map
  pcl::PointCloud<pcl::PointXYZI>::Ptr _filtered_input_cloud_global_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  voxelGridFilter(input_cloud_global_ptr, _filtered_input_cloud_global_ptr);
  _map += *_filtered_input_cloud_global_ptr;
  
  // publish map
  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(_map, map_msg);
  map_msg.header.frame_id = _map_frame_id;
  _map_pub.publish(map_msg);
   
  // filter input cloud, and register it as a last getted cloud
  voxelGridFilter(input_cloud_base_ptr, _filtered_previous_cloud_ptr);
  
  // previous_pose_change
  _previous_pose_change = pose_change;
}

void NDT_SLAM::voxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &out)
{
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setLeafSize(_voxel_leaf_size, _voxel_leaf_size, _voxel_leaf_size);
  filter.setInputCloud(in);
  filter.filter(*out); 
}

void NDT_SLAM::calucurateInitGuessPoseChange(Eigen::Matrix4f &init_guess_pose_change)
{
  Eigen::Translation3f t(_previous_pose_change(0,3),
                         _previous_pose_change(1,3),
                         0);
  Eigen::Affine3f at;
  at = t;
  init_guess_pose_change = at.matrix();
}

void NDT_SLAM::ndt(const pcl::PointCloud<pcl::PointXYZI>::Ptr      &source,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr      &target,
                   const Eigen::Matrix4f                           &init_guess_pose_change,
                         Eigen::Matrix4f                           &pose_change)
{
  _omp_ndt.setInputTarget(target);
  
  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  if(_method_type==0)
  {
    _ndt.setInputSource(source);
    _ndt.align(output_cloud, init_guess_pose_change);
    //fitness_score = ndt.getFitnessScore();
    pose_change = _ndt.getFinalTransformation();
    //has_converged = ndt.hasConverged();
    //final_num_iteration = ndt.getFinalNumIteration();
    //transformation_probability = ndt.getTransformationProbability();
  }
  else
  {
    _omp_ndt.setInputSource(source);
    _omp_ndt.align(output_cloud, init_guess_pose_change);
    //fitness_score = ndt.getFitnessScore();
    pose_change = _omp_ndt.getFinalTransformation();
    //has_converged = _omp_ndt.hasConverged();
    //final_num_iteration = _omp_ndt.getFinalNumIteration();
    //transformation_probability = ndt.getTransformationProbability();
  } 
}
*/

