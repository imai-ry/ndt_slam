#include <ndt_slam/NDT_SLAM.h>

NDT_SLAM::NDT_SLAM(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  // publisher & subscriber
  _map_pub = nh.advertise<sensor_msgs::PointCloud2>("ndt_map", 1000);
  _sub = nh.subscribe("pointcloud", 100000, &NDT_SLAM::callback, this);

  // get transform base(global) coordinate to lidar coordinate
  double x,y,z,roll,pitch,yaw;
  if(!(private_nh.getParam("tf_btol_x", x)) ||
     !(private_nh.getParam("tf_btol_y", y)) ||
     !(private_nh.getParam("tf_btol_z", z)) ||
     !(private_nh.getParam("tf_btol_roll", roll)) ||
     !(private_nh.getParam("tf_btol_pitch", pitch)) ||
     !(private_nh.getParam("tf_btol_yaw", yaw)) )
    ROS_BREAK();
  
  Eigen::Translation3f tl_btol(x, y, z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
  _tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
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
  _map.header.frame_id = "map";
  
  _diff_pose.x = 0; _diff_pose.y = 0; _diff_pose.z = 0; 
  _diff_pose.roll = 0; _diff_pose.pitch = 0; _diff_pose.yaw=0;
  _previous_pose.x=0;_previous_pose.y=0;_previous_pose.z=0;
  _previous_pose.roll=0;_previous_pose.pitch=0;_previous_pose.yaw=0;
  _added_pose.x = 0; _added_pose.y = 0; _added_pose.z = 0; 
  _added_pose.roll = 0; _added_pose.pitch = 0; _added_pose.yaw=0;
  _current_pose.x=0;_current_pose.y=0;_current_pose.z=0;
  _current_pose.roll=0;_current_pose.pitch=0;_current_pose.yaw=0;
  _ndt_pose.x=0;_ndt_pose.y=0;_ndt_pose.z=0;
  _ndt_pose.roll=0;_ndt_pose.pitch=0;_ndt_pose.yaw=0;
}


void NDT_SLAM::callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> scan, tmp;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
  
  //
  pcl::fromROSMsg(*input, tmp);
  double max_scan_range = 150;
  double min_scan_range = 0.4;
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (min_scan_range < r && r < max_scan_range)
    {
      scan.push_back(p);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  
  // if this is first map, register it to map directly
  if(_is_first_scan == true)
  {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, _tf_btol);
    _map += *transformed_scan_ptr;
    _is_first_scan = false;
  }

  // voxel grid filter on "input_cloud_lidar"
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(_voxel_leaf_size, _voxel_leaf_size, _voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(_map));
  
  // ndt setting
  if (_method_type == 0)
  {
    _ndt.setTransformationEpsilon(_trans_eps);
    _ndt.setStepSize(_step_size);
    _ndt.setResolution(_ndt_res);
    _ndt.setMaximumIterations(_max_iter);
    _ndt.setInputSource(filtered_scan_ptr);
  }
  else
  {
    _omp_ndt.setTransformationEpsilon(_trans_eps);
    _omp_ndt.setStepSize(_step_size);
    _omp_ndt.setResolution(_ndt_res);
    _omp_ndt.setMaximumIterations(_max_iter);
    _omp_ndt.setInputSource(filtered_scan_ptr);
  }
  
  if (_is_first_map == true)
  {
    if (_method_type == 0)
      _ndt.setInputTarget(map_ptr);
    else
      _omp_ndt.setInputTarget(map_ptr);
    _is_first_map = false;
  }
  
  // calucurate init_guess 
  _guess_pose.x = _previous_pose.x + _diff_pose.x;
  _guess_pose.y = _previous_pose.y + _diff_pose.y;
  _guess_pose.z = _previous_pose.z + _diff_pose.z;
  _guess_pose.roll = _previous_pose.roll;
  _guess_pose.pitch = _previous_pose.pitch; 
  _guess_pose.yaw = _previous_pose.yaw + _diff_pose.yaw;
  
  
  Eigen::AngleAxisf init_rotation_x(_guess_pose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(_guess_pose.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(_guess_pose.yaw, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f init_translation(_guess_pose.x, _guess_pose.y,_guess_pose.z);

  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * _tf_btol;
 
  // ndt matching
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  double fitness_score;
  bool has_converged;
  int final_num_iteration;
  if (_method_type == 0)
  {
    _ndt.align(*output_cloud, init_guess);
    fitness_score = _ndt.getFitnessScore();
    t_localizer = _ndt.getFinalTransformation();
    has_converged = _ndt.hasConverged();
    final_num_iteration = _ndt.getFinalNumIteration();
    //transformation_probability = _ndt.getTransformationProbability();
  }
  else
  {
    _omp_ndt.align(*output_cloud, init_guess);
    fitness_score = _omp_ndt.getFitnessScore();
    t_localizer = _omp_ndt.getFinalTransformation();
    has_converged = _omp_ndt.hasConverged();
    final_num_iteration = _omp_ndt.getFinalNumIteration();
  }
  
  // calucurate base_link
  t_base_link = t_localizer * _tf_ltob;
  
  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);
  
  //update pose
  _ndt_pose.x = t_base_link(0, 3);
  _ndt_pose.y = t_base_link(1, 3);
  _ndt_pose.z = t_base_link(2, 3);
  
  /* 
  Eigen::Matrix3f rot;
  rot(0,0)=t_base_link(0,0); rot(0,1)=t_base_link(0,1); rot(0,2)=t_base_link(0,2);
  rot(1,0)=t_base_link(1,0); rot(1,1)=t_base_link(1,1); rot(1,2)=t_base_link(1,2);
  rot(2,0)=t_base_link(2,0); rot(2,1)=t_base_link(2,1); rot(2,2)=t_base_link(2,2);
  Eigen::Vector3f euler = rot.eulerAngles(2, 1, 0);
  current_pose.roll = euler(2);  
  current_pose.pitch = euler(1);  
  current_pose.yaw = euler(0);
  */
  
  tf::Matrix3x3 mat_b;
  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));
  mat_b.getRPY(_ndt_pose.roll, _ndt_pose.pitch, _ndt_pose.yaw, 1);
  
  _current_pose.x = _ndt_pose.x;
  _current_pose.y = _ndt_pose.y;
  _current_pose.z = _ndt_pose.z;
  _current_pose.roll = _ndt_pose.roll;
  _current_pose.pitch = _ndt_pose.pitch;
  _current_pose.yaw = _ndt_pose.yaw;
  
  _diff_pose.x = _current_pose.x - _previous_pose.x;
  _diff_pose.y = _current_pose.y - _previous_pose.y;
  _diff_pose.z = _current_pose.z - _previous_pose.z;
  _diff_pose.yaw = calcDiffForRadian(_current_pose.yaw, _previous_pose.yaw);
  
  _previous_pose.x = _current_pose.x;
  _previous_pose.y = _current_pose.y;
  _previous_pose.z = _current_pose.z;
  _previous_pose.roll = _current_pose.roll;
  _previous_pose.pitch = _current_pose.pitch;
  _previous_pose.yaw = _current_pose.yaw;

  // Calculate the shift between added_pos and current_pos
  double shift = sqrt(pow(_current_pose.x - _added_pose.x, 2.0) + pow(_current_pose.y - _added_pose.y, 2.0));
  if(shift >= _scan_shift)
  {
    // register to a map 
    _map += *transformed_scan_ptr;
    _added_pose.x = _current_pose.x;
    _added_pose.y = _current_pose.y;
    _added_pose.z = _current_pose.z;
    _added_pose.roll = _current_pose.roll;
    _added_pose.pitch = _current_pose.pitch;
    _added_pose.yaw = _current_pose.yaw;
    
    if (_method_type == 0)
      _ndt.setInputTarget(map_ptr);
    else
      _omp_ndt.setInputTarget(map_ptr);
  }
  
  // publish map
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  _map_pub.publish(*map_msg_ptr);
  
  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "NDT has converged: " << has_converged << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << final_num_iteration << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

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



