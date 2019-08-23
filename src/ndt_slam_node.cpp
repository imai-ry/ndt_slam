#include "ndt_slam/NDT_SLAM.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_slam");
  ros::NodeHandle nh, private_nh("~");
  
  NDT_SLAM ndt_slam;
  ndt_slam.setup(nh, private_nh);
  ndt_slam.start();
  
  ros::spin();
}
