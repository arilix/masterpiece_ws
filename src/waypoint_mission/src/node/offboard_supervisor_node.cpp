#include "offboard_supervisor.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waypoint_mission::OffboardSupervisor>());
  rclcpp::shutdown();
  return 0;
}
