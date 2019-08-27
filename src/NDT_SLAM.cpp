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
  
  _previous_pose.x = 0; _previous_pose.y = 0; _previous_pose.z = 0;
  _previous_pose.roll = 0; _previous_pose.pitch = 0; _previous_pose.yaw = 0;  
  _diff_pose.x = 0; _diff_pose.y = 0; _diff_pose.z = 0;
  _diff_pose.roll = 0; _diff_pose.pitch = 0; _diff_pose.yaw = 0;  
  
  _map_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  
  if(!(_private_nh.getParam("map_frame_id", _map_frame_id)) ||
     !(_private_nh.getParam("scan_shift", _scan_shift))) 
    ROS_BREAK();
  
  // get transform base(global) coordinate to lidar coordinate
  double x,y,z,roll,pitch,yaw;
  if(!(_private_nh.getParam("tf_btol_x", x)) ||
     !(_private_nh.getParam("tf_btol_y", y)) ||
     !(_private_nh.getParam("tf_btol_z", z)) ||
     !(_private_nh.getParam("tf_btol_roll", roll)) ||
     !(_private_nh.getParam("tf_btol_pitch", pitch)) ||
     !(_private_nh.getParam("tf_btol_yaw", yaw)) )
    ROS_BREAK();
  
  _tf_btol = Translation3d(x,y,z)
             *AngleAxisf(yaw, Vector3d::UnitZ())
             *AngleAxisf(pitch, Vector3d::UnitY())
             *AngleAxisf(roll, Vector3d::UnitX());
  
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
  Eigen::Matrix4d init_guess, t_localizer;
  calucurateInitGuess(init_guess);
  ndt(filtered_input_cloud_lidar_ptr, _map_ptr, init_guess, t_localizer);
  
  // set pose
  Eigen::Matrix4d t_base_link;
  t_base_link = t_localizer * _tf_ltob.matrix();
  std::cout << t_base_link << std::endl;
  
  tf::Matrix3x3 mat_b;
  Pose current_pose, ndt_pose;
  ndt_pose.x = 0; ndt_pose.y = 0; ndt_pose.z = 0; ndt_pose.roll = 0; ndt_pose.pitch = 0; ndt_pose.yaw = 0; 
  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);
  ROS_INFO("aaaaaaaaa");
  
  /*
  ndt_pose.x = t_base_link(0,3);  // x
  ndt_pose.y = t_base_link(1,3);  // y
  ndt_pose.z = t_base_link(2,3);  // z
  current_pose.x = ndt_pose.x;  // x
  current_pose.y = ndt_pose.y;  // y
  current_pose.z = ndt_pose.z;  // z
  current_pose.roll = ndt_pose.roll;
  current_pose.pitch = ndt_pose.pitch;  
  current_pose.yaw = ndt_pose.yaw;  
  _diff_pose.x = current_pose.x - _previous_pose.x;
  _diff_pose.y = current_pose.y - _previous_pose.y;
  _diff_pose.z = current_pose.z - _previous_pose.z;
  _diff_pose.yaw = calcDiffForRadian(current_pose.yaw, _previous_pose.yaw);
  _previous_pose.x = current_pose.x;
  _previous_pose.y = current_pose.y;
  _previous_pose.z = current_pose.z;
  _previous_pose.roll = current_pose.roll;
  _previous_pose.pitch = current_pose.pitch;
  _previous_pose.yaw = current_pose.yaw;
  
  
  // register to a map & publish map
  pcl::transformPointCloud(*input_cloud_lidar_ptr, input_cloud_global, t_localizer);
  *_map_ptr += input_cloud_global;
  sensor_msgs::PointCloud2 map;
  pcl::toROSMsg(*_map_ptr, map);
  map.header.frame_id = _map_frame_id;
  _map_pub.publish(map);
*/

  ROS_INFO("get");

}

double NDT_SLAM::calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}


void NDT_SLAM::voxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &out,
                                     double                                leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.setInputCloud(in);
  filter.filter(*out); 
}

void NDT_SLAM::calucurateInitGuess(Eigen::Matrix4d &init_guess)
{
  Pose guess_pose;
  guess_pose.x = _previous_pose.x + _diff_pose.x;
  guess_pose.y = _previous_pose.y + _diff_pose.y;
  guess_pose.z = _previous_pose.z + _diff_pose.z;
  guess_pose.roll = _previous_pose.roll;
  guess_pose.pitch = _previous_pose.pitch;
  guess_pose.yaw = _previous_pose.yaw + _diff_pose.yaw;
  
  Eigen::Translation3d init_translation(guess_pose.x, guess_pose.y, guess_pose.z);
  Eigen::AngleAxisf init_rotation_x(guess_pose.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose.yaw, Eigen::Vector3d::UnitZ());
  init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * _tf_btol.matrix();
}

void NDT_SLAM::ndt(const pcl::PointCloud<pcl::PointXYZI>::Ptr      &source,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr      &target,
                   const Eigen::Matrix4d                           &init_guess,
                         Eigen::Matrix4d                           &t_localizer)
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

