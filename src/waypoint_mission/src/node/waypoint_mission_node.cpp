#include "waypoint_mission.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<waypoint_mission::WaypointMission>());
  rclcpp::shutdown();
  return 0;
}
