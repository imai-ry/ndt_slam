#include <ndt_slam/NDT_SLAM.h>

NDT_SLAM::NDT_SLAM(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  // publisher & subscriber
  _map_pub = nh.advertise<sensor_msgs::PointCloud2>("ndt_map", 1);
  _sub = nh.subscribe("pointcloud", 1000, &NDT_SLAM::callback, this);

  // get transform base(global) coordinate to lidar coordinate
  double x,y,z,roll,pitch,yaw;
  if(!(private_nh.getParam("tf_btol_x", x)) ||
     !(private_nh.getParam("tf_btol_y", y)) ||
     !(private_nh.getParam("tf_btol_z", z)) ||
     !(private_nh.getParam("tf_btol_roll", roll)) ||
     !(private_nh.getParam("tf_btol_pitch", pitch)) ||
     !(private_nh.getParam("tf_btol_yaw", yaw)) )
    ROS_BREAK();
  
  Eigen::Affine3d affine_btol;
  affine_btol = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                *Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                *Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                *Eigen::Translation3d(x,y,z);
  _tf_btol = affine_btol.matrix(); 
  _tf_ltob = _tf_btol.inverse();
      
  // get NDT parameter
  if(!(private_nh.getParam("method_type", _method_type)) ||
     !(private_nh.getParam("trans_eps", _trans_eps)) ||
     !(private_nh.getParam("step_size", _ndt_res)) ||
     !(private_nh.getParam("ndt_res", _ndt_res)) ||
     !(private_nh.getParam("max_iter", _max_iter)))
    ROS_BREAK();
  
  // other parameters
    if(!(private_nh.getParam("voxel_leaf_size", _voxel_leaf_size)) ||
       !(private_nh.getParam("scan_shift", _scan_shift))) 
    ROS_BREAK();
  
  // etc
  _is_first_scan = true; 
  _is_first_map = true;
  _diff_pose = Eigen::Vector3f::Zero();
  _map_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
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
  
  /*
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
  
  // register to a map 
  pcl::transformPointCloud(*input_cloud_lidar_ptr, input_cloud_global, t_localizer);
  *_map_ptr += input_cloud_global;

  */
  // publish map
  sensor_msgs::PointCloud2 map;
  pcl::toROSMsg(*_map_ptr, map);
  map.header.frame_id = "map";
  _map_pub.publish(map);
}

void NDT_SLAM::voxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &out,
                                     double                               voxel_size)
{
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  filter.setInputCloud(in);
  filter.filter(*out); 
}

/*

void NDT_SLAM::calucurateInitGuess(Eigen::Matrix4f &init_guess)
{
  Eigen::Vector3f guess_pose;
  guess_pose = _previous_pose + _diff_pose;
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
  if(is_first_map == true)
  {
    if(_method_type==0) _ndt.setInputTarget(target);
    else                _omp_ndt.setInputTarget(target);
    is_first_map = false;
  }
  
  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  if(_method_type==0)
  {
    _ndt.setTransformationEpsilon(_trans_eps);
    _ndt.setStepSize(_step_size);
    _ndt.setResolution(_ndt_res);
    _ndt.setMaximumIterations(_max_iter);
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
    _omp_ndt.setTransformationEpsilon(_trans_eps);
    _omp_ndt.setStepSize(_step_size);
    _omp_ndt.setResolution(_ndt_res);
    _omp_ndt.setMaximumIterations(_max_iter);
    _omp_ndt.setInputSource(source);
    _omp_ndt.align(output_cloud, init_guess);
    //fitness_score = ndt.getFitnessScore();
    t_localizer = _omp_ndt.getFinalTransformation();
    //has_converged = _omp_ndt.hasConverged();
    //final_num_iteration = _omp_ndt.getFinalNumIteration();
    //transformation_probability = ndt.getTransformationProbability();
  } 
}
*/


