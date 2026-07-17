#ifndef PX4_WAYPOINT_MISSION__WAYPOINT_MISSION_H_
#define PX4_WAYPOINT_MISSION__WAYPOINT_MISSION_H_

#include "px4_waypoint_mission/geofence.h"
#include "px4_waypoint_mission/waypoint.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace px4_waypoint_mission
{

class WaypointMission : public rclcpp::Node
{
public:
  WaypointMission();

private:
  enum class Phase {kWaitForOffboard, kTakeoffVertical, kAlignHeading, kNavigate, kHold, kFinished};

  void on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void on_home_position(const px4_msgs::msg::HomePosition::SharedPtr msg);
  void on_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
  void on_failsafe_flags(const px4_msgs::msg::FailsafeFlags::SharedPtr msg);
  void on_angular_velocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);
  void on_land_detected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
  void tick();
  void publish_target(const LocalWaypoint & waypoint);
  void publish_feedforward();
  bool waypoint_reached(const LocalWaypoint & waypoint) const;
  bool pass_condition_met(const LocalWaypoint & waypoint) const;
  std::optional<LocalWaypoint> resolve_waypoint(const GlobalWaypoint & waypoint) const;
  double target_heading(const LocalWaypoint & waypoint) const;
  double target_heading_for(const LocalWaypoint & from, const LocalWaypoint & to) const;
  double limited_yaw(double desired_yaw);
  bool yaw_rate_settled() const;
  void enter_alignment(const LocalWaypoint & waypoint);
  void begin_takeoff(const LocalWaypoint & takeoff_target);
  bool estimator_healthy() const;
  bool check_mission_feasibility();
  bool check_geofence(const GlobalWaypoint & waypoint, const LocalWaypoint & local,
    const std::optional<LocalWaypoint> & previous);
  void load_geofence();
  void check_estimator_resets();
  void check_yaw_alignment_watchdog();
  void check_takeoff_watchdog();
  void abort_mission(const std::string & reason);
  std::string canonical_mission_text() const;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr feedforward_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr abort_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr home_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_sub_;
  rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafe_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> local_position_;
  std::optional<px4_msgs::msg::VehicleStatus> vehicle_status_;
  std::optional<px4_msgs::msg::HomePosition> home_position_;
  std::optional<px4_msgs::msg::VehicleGlobalPosition> global_position_;
  std::optional<px4_msgs::msg::FailsafeFlags> failsafe_flags_;
  std::optional<px4_msgs::msg::VehicleAngularVelocity> angular_velocity_;
  std::optional<px4_msgs::msg::VehicleLandDetected> land_detected_;
  rclcpp::Time last_global_position_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_angular_velocity_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_land_detected_at_{0, 0, RCL_ROS_TIME};
  std::vector<GlobalWaypoint> waypoints_;
  std::optional<std::vector<LatLonPoint>> geofence_inclusion_;
  std::vector<std::vector<LatLonPoint>> geofence_exclusions_;
  std::size_t current_index_{0U};
  std::optional<rclcpp::Time> reached_since_;
  Phase phase_{Phase::kWaitForOffboard};
  LocalWaypoint takeoff_target_{};
  LocalWaypoint alignment_hold_{};
  double initial_yaw_{0.0};
  double segment_yaw_{0.0};
  double commanded_yaw_{0.0};
  double commanded_yaw_rate_rad_s_{0.0};
  bool yaw_rate_saturated_{false};
  bool yaw_initialized_{false};
  bool mission_feasible_{false};
  bool mission_aborted_{false};
  bool reset_counters_initialized_{false};
  uint8_t last_xy_reset_counter_{0};
  uint8_t last_z_reset_counter_{0};
  uint8_t last_heading_reset_counter_{0};
  uint8_t last_global_xy_reset_counter_{0};
  uint8_t last_global_alt_reset_counter_{0};
  uint32_t initial_home_update_count_{0};
  uint64_t last_ref_timestamp_{0};
  std::deque<rclcpp::Time> reset_event_times_;
  std::optional<rclcpp::Time> alignment_started_at_;
  std::optional<rclcpp::Time> yaw_progress_check_at_;
  double yaw_progress_last_abs_error_{0.0};
  std::optional<rclcpp::Time> takeoff_started_at_;
  std::optional<rclcpp::Time> takeoff_progress_check_at_;
  double takeoff_progress_last_down_error_{0.0};
  std::optional<rclcpp::Time> last_tick_time_;
  double acceptance_xy_m_;
  double acceptance_z_m_;
  double stopped_speed_mps_;
  double publish_rate_hz_;
  double yaw_acceptance_rad_;
  double max_yaw_rate_rad_s_;
  double max_yaw_accel_rad_s2_;
  double yaw_rate_stopped_rad_s_;
  double angular_velocity_timeout_s_;
  bool require_yaw_rate_feedback_;
  double yaw_alignment_timeout_s_;
  double yaw_stuck_window_s_;
  double yaw_stuck_min_progress_rad_;
  double takeoff_timeout_s_;
  double takeoff_stuck_window_s_;
  double takeoff_stuck_min_progress_m_;
  double land_detected_timeout_s_;
  double max_eph_m_;
  double max_epv_m_;
  double global_position_timeout_s_;
  double max_segment_length_m_;
  double max_distance_from_home_m_;
  double min_altitude_above_home_m_;
  double max_altitude_above_home_m_;
  double reset_storm_window_s_;
  int max_resets_per_window_;
  double geofence_margin_m_;
  double geofence_sample_resolution_m_;
  double fly_through_speed_mps_;
  double fly_through_pass_radius_m_;
  double fly_through_yaw_tolerance_rad_;
  bool require_offboard_;
  int64_t schema_version_;
};

}  // namespace px4_waypoint_mission

#endif  // PX4_WAYPOINT_MISSION__WAYPOINT_MISSION_H_
