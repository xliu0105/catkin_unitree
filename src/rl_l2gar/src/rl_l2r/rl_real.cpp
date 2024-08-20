#include "rl_l2gar/rl_l2r/rl_real_bridge.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rl_real_bridge");
  ros::NodeHandle nh;
  rl_real_bridge bridge(nh);

  bridge.main_func();

  return 0;
}
