#include <ros/ros.h>
#include <ndt_mapper/NDTMapper.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_mapper");
  ros::NodeHandle n("~");

  NDTMapper ndtm;

  if (!ndtm.initialize(n))
    {
      ROS_ERROR("%s: failed to initialize ndt mapper",
                ros::this_node::getName().c_str());
      return EXIT_FAILURE;
    }

  ros::spin();

  return EXIT_SUCCESS;
}
