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
  _is_first_map = true;
  
  _global_pose_change = Eigen::Matrix4f::Identity();

  _filtered_previous_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  
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
  Eigen::Translation3f t(0,0,0); //kari
  Eigen::Affine3f at;
  at = t;
  init_guess_pose_change = at.matrix();
}

void NDT_SLAM::ndt(const pcl::PointCloud<pcl::PointXYZI>::Ptr      &source,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr      &target,
                   const Eigen::Matrix4f                           &init_guess_pose_change,
                         Eigen::Matrix4f                           &pose_change)
{
  if(_is_first_map == true)
  {
    if(_method_type==0) _ndt.setInputTarget(target);
    else _omp_ndt.setInputTarget(target);
    _is_first_map = false;
  }
  
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












/*
void NDT_SLAM::callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{ 
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::fromROSMsg(*input, scan);
  
  // limit distanace
  
  // from lidar coordinate to base(robot) coordinate
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
    if(_method_type==0) _ndt.setInputTarget(_previous_cloud_ptr);
    else _omp_ndt.setInputTarget(_previous_cloud_ptr);

    _is_first_map = false;
  }
  
  Eigen::Matrix4f pose_change;
  if(_method_type==0)
  {
    _ndt.setInputSource(filtered_scan_ptr);
    _ndt.align(output_cloud, init_guess);
    //fitness_score = ndt.getFitnessScore();
    pose_change = _ndt.getFinalTransformation();
    //has_converged = ndt.hasConverged();
    //final_num_iteration = ndt.getFinalNumIteration();
    //transformation_probability = ndt.getTransformationProbability();
  }
  else
  {
    _omp_ndt.setInputSource(filtered_scan_ptr);
    _omp_ndt.align(output_cloud, init_guess);
    //fitness_score = ndt.getFitnessScore();
    pose_change = _omp_ndt.getFinalTransformation();
    //has_converged = _omp_ndt.hasConverged();
    //final_num_iteration = _omp_ndt.getFinalNumIteration();
    //transformation_probability = ndt.getTransformationProbability();
  } 

}
*/



/*
  // add scan to map
  _estimated_pose.x += pose_change(0,3);
  _estimated_pose.y += pose_change(1,3);
  float diff_x = _estimated_pose.x - _current_pose.x;
  float diff_y = _estimated_pose.y - _current_pose.y;
  float shift = std::sqrt(std::pow(diff_x, 2.0) + std::pow(diff_y, 2.0));
  ROS_INFO("shift %f", shift);
  if(_scan_shift < shift)
  {
    pcl::transformPointCloud(*transformed_scan_ptr, output_cloud, pose_change);
    *_map_ptr += output_cloud;
    
    // publish map
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*_map_ptr, map_msg);
    map_msg.header.frame_id = _map_frame_id;
    _map_pub.publish(map_msg);
    
    // update pose
    _current_pose.x = _estimated_pose.x;
    _current_pose.y = _estimated_pose.y;

*/



