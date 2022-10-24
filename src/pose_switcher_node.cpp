
#include "pose_switcher/pose_switcher.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "pose_switcher");
  PoseSwitcher ps;

  ros::spin();

  return 0;
};

