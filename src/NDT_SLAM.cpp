#include <ndt_slam/NDT_SLAM.h>

NDT_SLAM::NDT_SLAM(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  // publisher & subscriber
  _map_pub = nh.advertise<sensor_msgs::PointCloud2>("ndt_map", 1);
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
  
  Eigen::Affine3f affine_btol;
  affine_btol = Eigen::Translation3f(x,y,z)
                *Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
                *Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                *Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
  _tf_btol = affine_btol.matrix(); 
  _tf_ltob = _tf_btol.inverse();
      
  // get NDT parameter
  if(!(private_nh.getParam("method_type", _method_type)) ||
     !(private_nh.getParam("trans_eps", _trans_eps)) ||
     !(private_nh.getParam("step_size", _step_size)) ||
     !(private_nh.getParam("ndt_res", _ndt_res)) ||
     !(private_nh.getParam("max_iter", _max_iter)))
    ROS_BREAK();
  
  // other parameters
    if(!(private_nh.getParam("voxel_leaf_size", _voxel_leaf_size)) ||
       !(private_nh.getParam("scan_shift", _scan_shift)) ||
       !(private_nh.getParam("min_scan_range", _min_scan_range)) ||
       !(private_nh.getParam("max_scan_range", _max_scan_range)) 
       ) 
    ROS_BREAK();
  
  // etc
  _is_first_scan = true; 
  _is_first_map = true;
  _map_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  
  _diff_pose.x = 0; _diff_pose.y = 0; _diff_pose.z = 0; 
  _diff_pose.roll = 0; _diff_pose.pitch = 0; _diff_pose.yaw=0;
  _previous_pose.x=0;_previous_pose.y=0;_previous_pose.z=0;
  _previous_pose.roll=0;_previous_pose.pitch=0;_previous_pose.yaw=0;
  _added_pose.x = 0; _added_pose.y = 0; _added_pose.z = 0; 
  _added_pose.roll = 0; _added_pose.pitch = 0; _added_pose.yaw=0;
  
}

void NDT_SLAM::test_1()
{
  std::cout << "########## parameter ##########" << std::endl;
  std::cout << "_tf_btol : " << std::endl;
  std::cout << _tf_btol << std::endl;
  std::cout << "_tf_ltob : " << std::endl;
  std::cout << _tf_ltob << std::endl;
  std::cout << "_method_type : " << _method_type << std::endl;
  std::cout << "_trans_eps : " << _trans_eps << std::endl;
  std::cout << "_step_size : " << _step_size << std::endl;
  std::cout << "_ndt_res : " << _ndt_res << std::endl;
  std::cout << "_max_iter : " << _max_iter << std::endl;
  std::cout << "_voxel_leaf_size : " << _voxel_leaf_size << std::endl;
  std::cout << "_scan_shift : " << _scan_shift << std::endl;
  std::cout << "_is_first_scan : " << _is_first_scan << std::endl;
  std::cout << "_is_first_map: " << _is_first_map << std::endl;
  std::cout << "########################" << std::endl;
}

void NDT_SLAM::pointCloudRangeFilter(const pcl::PointCloud<pcl::PointXYZI> &in, pcl::PointCloud<pcl::PointXYZI> &out) const
{
  pcl::PointXYZI p;
  for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = in.begin(); item != in.end(); item++)
  {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    double r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (_min_scan_range < r && r < _max_scan_range)
    {
      out.push_back(p);
    }
  }
}


void NDT_SLAM::callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  //debug
  test_1();

  // range filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_lidar_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_lidar_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*input, *input_cloud_lidar_ptr);
  
  ros::Time start1 = ros::Time::now();
  pointCloudRangeFilter(*input_cloud_lidar_ptr, *scan_lidar_ptr);
  ros::Time end1 = ros::Time::now();
  ROS_INFO("time1 %f", (end1-start1).toSec());  
    
  // if this is first map, register it to map directly
  pcl::PointCloud<pcl::PointXYZI>      scan_global;
  if(_is_first_scan == true)
  {
    pcl::transformPointCloud(*scan_lidar_ptr, scan_global, _tf_btol);
    *_map_ptr += scan_global;
    _previous_scan = scan_global; 
    _is_first_scan = false;
  }

  // voxel grid filter on "input_cloud_lidar"
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_lidar_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  voxelGridFilter(scan_lidar_ptr, filtered_scan_lidar_ptr, _voxel_leaf_size);
  
  // calucurate init_guess
  Eigen::Matrix4f init_guess, t_localizer;
  
  Pose guess_pose;
  guess_pose.x = _previous_pose.x + _diff_pose.x;
  guess_pose.y = _previous_pose.y + _diff_pose.y;
  guess_pose.z = _previous_pose.z + _diff_pose.z;
  guess_pose.roll = _previous_pose.roll;
  guess_pose.pitch = _previous_pose.pitch; 
  guess_pose.yaw = _previous_pose.yaw + _diff_pose.yaw;
  
  Eigen::Affine3f af;
  af = Eigen::Translation3f(guess_pose.x, guess_pose.y, guess_pose.z)
       *Eigen::AngleAxisf(guess_pose.yaw, Eigen::Vector3f::UnitZ())
       *Eigen::AngleAxisf(guess_pose.pitch, Eigen::Vector3f::UnitY())
       *Eigen::AngleAxisf(guess_pose.roll, Eigen::Vector3f::UnitX());
  init_guess = af.matrix() * _tf_btol;
 
  // ndt matching
  ros::Time start2 = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr previous_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(_previous_scan));
  ndt(scan_lidar_ptr, previous_scan_ptr, init_guess, t_localizer);
  ros::Time end2 = ros::Time::now();
  ROS_INFO("time2 %f", (end2-start2).toSec());  
  
  // update pose 
  Eigen::Matrix4f t_base_link;
  t_base_link = t_localizer * _tf_ltob;
  
  Pose current_pose;
  current_pose.x=0;current_pose.y=0;current_pose.z=0;current_pose.roll=0;current_pose.pitch=0;current_pose.yaw=0;
  current_pose.x = t_base_link(0,3);  
  current_pose.y = t_base_link(1,3);  
  current_pose.z = t_base_link(2,3);
  
  Eigen::Matrix3f rot;
  rot(0,0)=t_base_link(0,0); rot(0,1)=t_base_link(0,1); rot(0,2)=t_base_link(0,2);
  rot(1,0)=t_base_link(1,0); rot(1,1)=t_base_link(1,1); rot(1,2)=t_base_link(1,2);
  rot(2,0)=t_base_link(2,0); rot(2,1)=t_base_link(2,1); rot(2,2)=t_base_link(2,2);
  //Eigen::Vector3f euler = rot.eulerAngles(2, 1, 0);
  //current_pose.roll = euler(2);  
  //current_pose.pitch = euler(1);  
  //current_pose.yaw = euler(0);
  std::cout << "t_base_link" << std::endl;
  std::cout << t_base_link << std::endl;
  std::cout << "rot" << std::endl;
  std::cout << rot << std::endl;
  //std::cout << "euler" << std::endl;
  //std::cout << euler << std::endl;
  
  tf::Matrix3x3 mat_b;
  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));
  mat_b.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw, 1);
  std::cout << "tf ypr" << std::endl;
  std::cout << current_pose.yaw << std::endl;
  std::cout << current_pose.pitch << std::endl;
  std::cout << current_pose.roll << std::endl;

  
  _diff_pose.x = current_pose.x - _previous_pose.x;
  _diff_pose.y = current_pose.y - _previous_pose.y;
  _diff_pose.z = current_pose.z - _previous_pose.z;
  _diff_pose.yaw = calcDiffForRadian(current_pose.yaw, _previous_pose.yaw);
  
  double shift = sqrt(pow(current_pose.x - _added_pose.x, 2.0) + pow(current_pose.y - _added_pose.y, 2.0));
  if(shift > _scan_shift)
  {
    // register to a map 
    pcl::transformPointCloud(*scan_lidar_ptr, scan_global, t_localizer);
    *_map_ptr += scan_global;
    
    _added_pose.x = current_pose.x;
    _added_pose.y = current_pose.y;
    _added_pose.z = current_pose.z;
    _added_pose.roll = current_pose.roll;
    _added_pose.pitch = current_pose.pitch;
    _added_pose.yaw = current_pose.yaw;
    
    _previous_scan = scan_global;
  }
  
  // publish map
  sensor_msgs::PointCloud2 map;
  pcl::toROSMsg(*_map_ptr, map);
  map.header.frame_id = "map";
  _map_pub.publish(map);
  
  _previous_pose.x = current_pose.x;
  _previous_pose.y = current_pose.y;
  _previous_pose.z = current_pose.z;
  _previous_pose.roll = current_pose.roll;
  _previous_pose.pitch = current_pose.pitch;
  _previous_pose.yaw = current_pose.yaw;
  
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

void NDT_SLAM::ndt(const pcl::PointCloud<pcl::PointXYZI>::Ptr      &source,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr      &target,
                   const Eigen::Matrix4f                           &init_guess,
                   Eigen::Matrix4f                           &t_localizer)
{
  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  if(_method_type==0)
  {
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setInputTarget(target);
    ndt.setTransformationEpsilon(_trans_eps);
    ndt.setStepSize(_step_size);
    ndt.setResolution(_ndt_res);
    ndt.setMaximumIterations(_max_iter);
    ndt.setInputSource(source);
    ndt.align(output_cloud, init_guess);
    //fitness_score = ndt.getFitnessScore();
    t_localizer = ndt.getFinalTransformation();
    //has_converged = ndt.hasConverged();
    //final_num_iteration = ndt.getFinalNumIteration();
    //transformation_probability = ndt.getTransformationProbability();
  }
  else
  {
    pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> omp_ndt;
    omp_ndt.setInputTarget(target);
    omp_ndt.setTransformationEpsilon(_trans_eps);
    omp_ndt.setStepSize(_step_size);
    omp_ndt.setResolution(_ndt_res);
    omp_ndt.setMaximumIterations(_max_iter);
    omp_ndt.setInputSource(source);
    omp_ndt.align(output_cloud, init_guess);
    //fitness_score = ndt.getFitnessScore();
    t_localizer = omp_ndt.getFinalTransformation();
    //has_converged = _omp_ndt.hasConverged();
    //final_num_iteration = _omp_ndt.getFinalNumIteration();
    //transformation_probability = ndt.getTransformationProbability();
  } 
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



