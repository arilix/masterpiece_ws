#include "px4_waypoint_mission/offboard_supervisor.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_waypoint_mission::OffboardSupervisor>());
  rclcpp::shutdown();
  return 0;
}
