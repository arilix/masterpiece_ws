#include "waypoint_mission.h"
#include "sha256.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace waypoint_mission
{

namespace
{
const auto kPx4Qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
constexpr double kPi = 3.14159265358979323846;

double wrap_pi(double angle)
{
  while (angle > kPi) {angle -= 2.0 * kPi;}
  while (angle < -kPi) {angle += 2.0 * kPi;}
  return angle;
}
}

WaypointMission::WaypointMission()
: Node("waypoint_mission")
{
  schema_version_ = declare_parameter<int64_t>("schema_version", 1);
  if (schema_version_ != 1) {
    throw std::runtime_error("schema_version tidak didukung; workspace ini hanya memahami versi 1");
  }
  acceptance_xy_m_ = declare_parameter("acceptance_xy_m", 0.5);
  waypoint_forward_bias_m_ = declare_parameter("waypoint_forward_bias_m", 0.2);
  max_altitude_error_m_ = declare_parameter("max_altitude_error_m", 3.0);
  acceptance_z_m_ = declare_parameter("acceptance_z_m", 0.4);
  stopped_speed_mps_ = declare_parameter("stopped_speed_mps", 0.3);
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 10.0);
  yaw_acceptance_rad_ = declare_parameter("yaw_acceptance_rad", 0.12);
  max_yaw_rate_rad_s_ = declare_parameter("max_yaw_rate_rad_s", 0.7);
  max_yaw_accel_rad_s2_ = declare_parameter("max_yaw_accel_rad_s2", 0.6);
  yaw_rate_stopped_rad_s_ = declare_parameter("yaw_rate_stopped_rad_s", 0.06);
  angular_velocity_timeout_s_ = declare_parameter("angular_velocity_timeout_s", 0.5);
  require_yaw_rate_feedback_ = declare_parameter("require_yaw_rate_feedback", false);
  yaw_alignment_timeout_s_ = declare_parameter("yaw_alignment_timeout_s", 20.0);
  yaw_stuck_window_s_ = declare_parameter("yaw_stuck_window_s", 3.0);
  yaw_stuck_min_progress_rad_ = declare_parameter("yaw_stuck_min_progress_rad", 0.02);
  takeoff_timeout_s_ = declare_parameter("takeoff_timeout_s", 30.0);
  takeoff_stuck_window_s_ = declare_parameter("takeoff_stuck_window_s", 4.0);
  takeoff_stuck_min_progress_m_ = declare_parameter("takeoff_stuck_min_progress_m", 0.08);
  land_detected_timeout_s_ = declare_parameter("land_detected_timeout_s", 1.0);
  max_eph_m_ = declare_parameter("max_eph_m", 2.0);
  max_epv_m_ = declare_parameter("max_epv_m", 3.0);
  global_position_timeout_s_ = declare_parameter("global_position_timeout_s", 1.0);
  max_segment_length_m_ = declare_parameter("max_segment_length_m", 500.0);
  max_distance_from_home_m_ = declare_parameter("max_distance_from_home_m", 1000.0);
  min_altitude_above_home_m_ = declare_parameter("min_altitude_above_home_m", 2.0);
  max_altitude_above_home_m_ = declare_parameter("max_altitude_above_home_m", 120.0);
  reset_storm_window_s_ = declare_parameter("reset_storm_window_s", 30.0);
  max_resets_per_window_ = static_cast<int>(
    declare_parameter<int64_t>("max_resets_per_window", 4));
  fly_through_speed_mps_ = declare_parameter("fly_through_speed_mps", 1.5);
  fly_through_pass_radius_m_ = declare_parameter("fly_through_pass_radius_m", 1.5);
  fly_through_yaw_tolerance_rad_ = declare_parameter("fly_through_yaw_tolerance_rad", 0.35);
  require_offboard_ = declare_parameter("require_offboard", true);
  use_tf_mini_altitude_ = declare_parameter("use_tf_mini_altitude", false);
  tf_mini_timeout_s_ = declare_parameter("tf_mini_timeout_s", 0.5);
  tf_mini_climb_rate_mps_ = declare_parameter("tf_mini_climb_rate_mps", 0.3);
  tf_mini_range_topic_ = declare_parameter("tf_mini_range_topic", std::string{"/range"});
  vehicle_status_topic_ = declare_parameter(
    "vehicle_status_topic", std::string{"/fmu/out/vehicle_status_v1"});
  const auto waypoint_text = declare_parameter<std::vector<std::string>>(
    "waypoints", std::vector<std::string>{});
  if (waypoint_text.empty() || waypoint_text.size() > 10U) {
    throw std::runtime_error("Parameter 'waypoints' wajib 1..10 titik aktif");
  }
  for (const auto & text : waypoint_text) {
    waypoints_.push_back(parse_waypoint(text));
  }
  load_geofence();
  if (acceptance_xy_m_ <= 0.0 || waypoint_forward_bias_m_ < 0.0 ||
    max_altitude_error_m_ <= 0.0 || acceptance_z_m_ <= 0.0 ||
    stopped_speed_mps_ < 0.0 ||
    publish_rate_hz_ < 2.0 || yaw_acceptance_rad_ <= 0.0 || max_yaw_rate_rad_s_ <= 0.0 ||
    max_eph_m_ <= 0.0 || max_epv_m_ <= 0.0 || global_position_timeout_s_ <= 0.0 ||
    max_segment_length_m_ <= 0.0 || max_distance_from_home_m_ <= 0.0 ||
    min_altitude_above_home_m_ < 0.0 ||
    max_altitude_above_home_m_ <= min_altitude_above_home_m_)
  {
    throw std::runtime_error("Parameter acceptance/speed/rate/yaw tidak valid");
  }
  if (max_yaw_accel_rad_s2_ <= 0.0 || yaw_rate_stopped_rad_s_ <= 0.0 ||
    angular_velocity_timeout_s_ <= 0.0 || yaw_alignment_timeout_s_ <= 0.0 ||
    yaw_stuck_window_s_ <= 0.0 || yaw_stuck_min_progress_rad_ < 0.0 ||
    takeoff_timeout_s_ <= 0.0 || takeoff_stuck_window_s_ <= 0.0 ||
    takeoff_stuck_min_progress_m_ < 0.0 || land_detected_timeout_s_ <= 0.0 ||
    reset_storm_window_s_ <= 0.0 || max_resets_per_window_ < 1 ||
    fly_through_speed_mps_ <= 0.0 || fly_through_pass_radius_m_ <= 0.0 ||
    fly_through_yaw_tolerance_rad_ <= 0.0 || tf_mini_timeout_s_ <= 0.0 ||
    tf_mini_climb_rate_mps_ <= 0.0)
  {
    throw std::runtime_error("Parameter watchdog/yaw-shaping/fly-through tidak valid");
  }

  target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mission/target_ned", rclcpp::QoS(1).reliable());
  feedforward_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    "/mission/target_feedforward", rclcpp::QoS(1).reliable());
  state_pub_ = create_publisher<std_msgs::msg::UInt32>(
    "/mission/current_waypoint", rclcpp::QoS(1).reliable().transient_local());
  abort_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/mission/abort", rclcpp::QoS(1).reliable().transient_local());
  land_request_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/mission/land_request", rclcpp::QoS(1).reliable().transient_local());
  arm_request_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/mission/arm_request", rclcpp::QoS(1).reliable().transient_local());
  local_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", kPx4Qos,
    std::bind(&WaypointMission::on_local_position, this, std::placeholders::_1));
  status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    vehicle_status_topic_, kPx4Qos,
    std::bind(&WaypointMission::on_vehicle_status, this, std::placeholders::_1));
  global_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
    "/fmu/out/vehicle_global_position", kPx4Qos,
    std::bind(&WaypointMission::on_global_position, this, std::placeholders::_1));
  failsafe_sub_ = create_subscription<px4_msgs::msg::FailsafeFlags>(
    "/fmu/out/failsafe_flags", kPx4Qos,
    std::bind(&WaypointMission::on_failsafe_flags, this, std::placeholders::_1));
  // Opsional: jika require_yaw_rate_feedback=true, topic ini dipakai untuk
  // membuktikan yaw-rate aktual sudah tenang sebelum translasi dimulai.
  angular_velocity_sub_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
    "/fmu/out/vehicle_angular_velocity", kPx4Qos,
    std::bind(&WaypointMission::on_angular_velocity, this, std::placeholders::_1));
  land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
    "/fmu/out/vehicle_land_detected", kPx4Qos,
    std::bind(&WaypointMission::on_land_detected, this, std::placeholders::_1));
  if (use_tf_mini_altitude_) {
    range_sub_ = create_subscription<sensor_msgs::msg::Range>(
      tf_mini_range_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&WaypointMission::on_range, this, std::placeholders::_1));
    RCLCPP_WARN(get_logger(),
      "tf_mini altitude gate aktif: membaca sensor_msgs/Range dari %s untuk cek altitude",
      tf_mini_range_topic_.c_str());
  }
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&WaypointMission::tick, this));
  RCLCPP_INFO(get_logger(), "Mission dimuat: %zu waypoint global LLA", waypoints_.size());
  RCLCPP_INFO(get_logger(), "Mission hash (SHA-256): %s",
    Sha256::hash_hex(canonical_mission_text()).c_str());
  if (!require_yaw_rate_feedback_) {
    RCLCPP_WARN(get_logger(),
      "require_yaw_rate_feedback=false: alignment memakai heading estimator dan yaw setpoint internal tanpa menunggu gyro yaw-rate aktual.");
  }
}

void WaypointMission::on_local_position(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  local_position_ = *msg;
}

void WaypointMission::on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  vehicle_status_ = *msg;
}

void WaypointMission::on_global_position(
  const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
  global_position_ = *msg;
  last_global_position_at_ = now();
}

void WaypointMission::on_failsafe_flags(const px4_msgs::msg::FailsafeFlags::SharedPtr msg)
{
  failsafe_flags_ = *msg;
}

void WaypointMission::on_angular_velocity(
  const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
{
  angular_velocity_ = *msg;
  last_angular_velocity_at_ = now();
}

void WaypointMission::on_land_detected(
  const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
  land_detected_ = *msg;
  last_land_detected_at_ = now();
}

void WaypointMission::on_range(const sensor_msgs::msg::Range::SharedPtr msg)
{
  range_ = *msg;
  last_range_at_ = now();
}

void WaypointMission::abort_mission(const std::string & reason)
{
  if (!mission_aborted_) {
    RCLCPP_ERROR(get_logger(), "MISSION ABORT: %s", reason.c_str());
    mission_aborted_ = true;
  }
  std_msgs::msg::Bool abort{};
  abort.data = true;
  abort_pub_->publish(abort);
}

std::string WaypointMission::canonical_mission_text() const
{
  std::ostringstream out;
  out << "schema_version=" << schema_version_ << ';';
  for (const auto & wp : waypoints_) {
    out << std::fixed << std::setprecision(8) << wp.latitude_deg << ',' << wp.longitude_deg
        << ',' << std::setprecision(3) << wp.altitude_m << ','
        << static_cast<int>(wp.altitude_frame) << ',' << std::setprecision(6)
        << (std::isfinite(wp.yaw_rad) ? wp.yaw_rad : 9999.0) << ',' << wp.hold_s << ','
        << (wp.fly_through ? 1 : 0) << ';';
  }
  return out.str();
}

void WaypointMission::load_geofence()
{
  const auto inclusion_text = declare_parameter("geofence_inclusion", std::string{});
  const auto exclusion_texts = declare_parameter<std::vector<std::string>>(
    "geofence_exclusions", std::vector<std::string>{});
  geofence_margin_m_ = declare_parameter("geofence_margin_m", 5.0);
  geofence_sample_resolution_m_ = declare_parameter("geofence_sample_resolution_m", 5.0);
  if (geofence_margin_m_ < 0.0 || geofence_sample_resolution_m_ <= 0.0) {
    throw std::runtime_error("Parameter geofence_margin_m/geofence_sample_resolution_m tidak valid");
  }
  const auto parse_polygon = [](const std::string & text) {
      std::vector<LatLonPoint> polygon;
      std::stringstream stream(text);
      std::string token;
      while (std::getline(stream, token, ';')) {
        const auto trimmed = trim(token);
        if (trimmed.empty()) {continue;}
        polygon.push_back(parse_latlon_point(trimmed));
      }
      return polygon;
    };
  if (!inclusion_text.empty()) {
    auto polygon = parse_polygon(inclusion_text);
    if (polygon.size() < 3U) {
      throw std::runtime_error("geofence_inclusion wajib minimal 3 titik jika diisi");
    }
    geofence_inclusion_ = std::move(polygon);
  }
  for (const auto & text : exclusion_texts) {
    auto polygon = parse_polygon(text);
    if (polygon.size() < 3U) {
      throw std::runtime_error("Setiap entri geofence_exclusions wajib minimal 3 titik");
    }
    geofence_exclusions_.push_back(std::move(polygon));
  }
}

bool WaypointMission::check_geofence(
  const GlobalWaypoint & /*waypoint*/, const LocalWaypoint & local,
  const std::optional<LocalWaypoint> & previous)
{
  if (!geofence_inclusion_.has_value() && geofence_exclusions_.empty()) {return true;}
  const PlanarPoint point{local.north_m, local.east_m};
  std::optional<PlanarPolygon> inclusion_planar;
  if (geofence_inclusion_.has_value()) {
    inclusion_planar = project_polygon(
      *geofence_inclusion_, local_position_->ref_lat, local_position_->ref_lon);
  }
  std::vector<PlanarPolygon> exclusions_planar;
  exclusions_planar.reserve(geofence_exclusions_.size());
  for (const auto & polygon : geofence_exclusions_) {
    exclusions_planar.push_back(
      project_polygon(polygon, local_position_->ref_lat, local_position_->ref_lon));
  }
  auto failure = check_point_against_fence(point, inclusion_planar, exclusions_planar,
    geofence_margin_m_);
  if (failure.has_value()) {
    abort_mission("geofence waypoint gagal: " + *failure);
    return false;
  }
  if (previous.has_value()) {
    const PlanarPoint from{previous->north_m, previous->east_m};
    failure = check_segment_against_fence(
      from, point, inclusion_planar, exclusions_planar, geofence_margin_m_,
      geofence_sample_resolution_m_);
    if (failure.has_value()) {
      abort_mission("geofence segmen gagal: " + *failure);
      return false;
    }
  }
  return true;
}

bool WaypointMission::estimator_healthy() const
{
  if (!local_position_.has_value() || !global_position_.has_value() ||
    (now() - last_global_position_at_).seconds() > global_position_timeout_s_)
  {
    return false;
  }
  const auto & local = *local_position_;
  const auto & global = *global_position_;
  if (!local.xy_valid || !local.z_valid || !local.v_xy_valid || !local.v_z_valid ||
    !local.xy_global || !local.z_global || local.dead_reckoning ||
    !global.lat_lon_valid || !global.alt_valid || global.dead_reckoning ||
    !std::isfinite(global.eph) || !std::isfinite(global.epv) ||
    global.eph > max_eph_m_ || global.epv > max_epv_m_)
  {
    return false;
  }
  if (failsafe_flags_.has_value() &&
    (failsafe_flags_->local_position_invalid || failsafe_flags_->global_position_invalid ||
    failsafe_flags_->home_position_invalid || failsafe_flags_->geofence_breached ||
    failsafe_flags_->local_position_accuracy_low || failsafe_flags_->navigator_failure))
  {
    return false;
  }
  return true;
}

bool WaypointMission::check_mission_feasibility()
{
  // /fmu/out/home_position tidak dibridge oleh firmware/dds_topics.yaml ini
  // (topic terdaftar tapi tidak pernah publish satu pesan pun -- diverifikasi
  // live). Referensi origin LLA dipakai dari VehicleLocalPosition.ref_lat/
  // ref_lon/ref_alt (EKF origin) sebagai gantinya; field ini valid selama
  // xy_global/z_global true dan sudah terbukti stream terus-menerus.
  if (!local_position_.has_value() || !local_position_->xy_global || !local_position_->z_global ||
    !std::isfinite(local_position_->ref_lat) || !std::isfinite(local_position_->ref_lon) ||
    !std::isfinite(local_position_->ref_alt))
  {
    return false;
  }
  std::optional<LocalWaypoint> previous;
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    if (waypoints_[index].altitude_frame == AltitudeFrame::kTerrain) {
      abort_mission(
        "waypoint memakai alt_frame TERRAIN; belum ada sumber DEM/datum tervalidasi (GAP §8.2)");
      return false;
    }
    if (waypoints_[index].fly_through && index + 1U >= waypoints_.size()) {
      abort_mission("waypoint terakhir tidak boleh FLY_THROUGH");
      return false;
    }
    const auto local = resolve_waypoint(waypoints_[index]);
    if (!local.has_value()) {return false;}
    const double altitude_above_home =
      waypoints_[index].altitude_frame == AltitudeFrame::kRelativeHome ?
      waypoints_[index].altitude_m : waypoints_[index].altitude_m - local_position_->ref_alt;
    if (altitude_above_home < min_altitude_above_home_m_ ||
      altitude_above_home > max_altitude_above_home_m_)
    {
      abort_mission("altitude waypoint di luar batas feasibility");
      return false;
    }
    GlobalWaypoint home_relative = waypoints_[index];
    const auto from_home = project_global_to_local(
      home_relative, local_position_->ref_lat, local_position_->ref_lon, local_position_->ref_alt,
      local_position_->ref_alt);
    if (std::hypot(from_home.north_m, from_home.east_m) > max_distance_from_home_m_) {
      abort_mission("waypoint melewati max_distance_from_home_m");
      return false;
    }
    if (previous.has_value() &&
      std::hypot(local->north_m - previous->north_m, local->east_m - previous->east_m) >
      max_segment_length_m_)
    {
      abort_mission("panjang segmen waypoint melewati batas");
      return false;
    }
    if (!check_geofence(waypoints_[index], *local, previous)) {
      return false;
    }
    previous = local;
  }
  initial_ref_timestamp_ = local_position_->ref_timestamp;
  last_ref_timestamp_ = local_position_->ref_timestamp;
  RCLCPP_INFO(get_logger(), "Mission feasibility PASSED: %zu waypoint, geofence %s",
    waypoints_.size(),
    (geofence_inclusion_.has_value() || !geofence_exclusions_.empty()) ? "aktif" : "tidak dikonfigurasi");
  return true;
}

void WaypointMission::check_estimator_resets()
{
  if (!reset_counters_initialized_) {
    last_xy_reset_counter_ = local_position_->xy_reset_counter;
    last_z_reset_counter_ = local_position_->z_reset_counter;
    last_heading_reset_counter_ = local_position_->heading_reset_counter;
    last_global_xy_reset_counter_ = global_position_->lat_lon_reset_counter;
    last_global_alt_reset_counter_ = global_position_->alt_reset_counter;
    last_ref_timestamp_ = local_position_->ref_timestamp;
    reset_counters_initialized_ = true;
    return;
  }
  const bool xy_local_reset = last_xy_reset_counter_ != local_position_->xy_reset_counter;
  const bool z_local_reset = last_z_reset_counter_ != local_position_->z_reset_counter;
  const bool heading_reset =
    last_heading_reset_counter_ != local_position_->heading_reset_counter;
  const bool global_latlon_reset =
    last_global_xy_reset_counter_ != global_position_->lat_lon_reset_counter;
  const bool global_alt_reset =
    last_global_alt_reset_counter_ != global_position_->alt_reset_counter;
  const bool reference_changed = last_ref_timestamp_ != local_position_->ref_timestamp;

  last_xy_reset_counter_ = local_position_->xy_reset_counter;
  last_z_reset_counter_ = local_position_->z_reset_counter;
  last_heading_reset_counter_ = local_position_->heading_reset_counter;
  last_global_xy_reset_counter_ = global_position_->lat_lon_reset_counter;
  last_global_alt_reset_counter_ = global_position_->alt_reset_counter;
  last_ref_timestamp_ = local_position_->ref_timestamp;

  const bool position_reset = xy_local_reset || z_local_reset || global_latlon_reset ||
    global_alt_reset || reference_changed;
  if (!position_reset && !heading_reset) {return;}

  RCLCPP_WARN(get_logger(),
    "Estimator reset [xy_local=%d z_local=%d heading=%d global_latlon=%d global_alt=%d "
    "ref_changed=%d] delta_xy=[%.3f,%.3f] delta_z=%.3f delta_heading=%.3f pos=[%.3f,%.3f,%.3f]",
    xy_local_reset, z_local_reset, heading_reset, global_latlon_reset, global_alt_reset,
    reference_changed, static_cast<double>(local_position_->delta_xy[0]),
    static_cast<double>(local_position_->delta_xy[1]),
    static_cast<double>(local_position_->delta_z),
    static_cast<double>(local_position_->delta_heading), local_position_->x, local_position_->y,
    local_position_->z);

  const auto event_time = now();
  reset_event_times_.push_back(event_time);
  while (!reset_event_times_.empty() &&
    (event_time - reset_event_times_.front()).seconds() > reset_storm_window_s_)
  {
    reset_event_times_.pop_front();
  }
  if (static_cast<int>(reset_event_times_.size()) > max_resets_per_window_) {
    abort_mission("reset estimator terlalu sering dalam window (reset storm)");
    return;
  }

  reached_since_.reset();
  const auto current = resolve_waypoint(waypoints_[current_index_]);
  if (current.has_value() && phase_ != Phase::kWaitForOffboard &&
    phase_ != Phase::kTakeoffVertical && phase_ != Phase::kFinished)
  {
    commanded_yaw_ = local_position_->heading;
    commanded_yaw_rate_rad_s_ = 0.0;
    enter_alignment(*current);
  }
}

void WaypointMission::check_yaw_alignment_watchdog()
{
  if (!alignment_started_at_.has_value()) {return;}
  const auto current_time = now();
  const double abs_error = std::abs(wrap_pi(segment_yaw_ - local_position_->heading));
  if ((current_time - *alignment_started_at_).seconds() > yaw_alignment_timeout_s_) {
    abort_mission("yaw alignment timeout (YAW_STUCK)");
    return;
  }
  if (!yaw_progress_check_at_.has_value()) {
    yaw_progress_check_at_ = current_time;
    yaw_progress_last_abs_error_ = abs_error;
    return;
  }
  if ((current_time - *yaw_progress_check_at_).seconds() >= yaw_stuck_window_s_) {
    const double progress = yaw_progress_last_abs_error_ - abs_error;
    if (progress < yaw_stuck_min_progress_rad_) {
      abort_mission("yaw alignment macet: progress heading di bawah minimum (YAW_STUCK)");
      return;
    }
    yaw_progress_check_at_ = current_time;
    yaw_progress_last_abs_error_ = abs_error;
  }
}

void WaypointMission::check_takeoff_watchdog()
{
  if (!takeoff_started_at_.has_value()) {return;}
  const auto current_time = now();
  const double down_error = altitude_error_m(takeoff_target_);
  if ((current_time - *takeoff_started_at_).seconds() > takeoff_timeout_s_) {
    const bool land_detector_fresh = land_detected_.has_value() &&
      (current_time - last_land_detected_at_).seconds() <= land_detected_timeout_s_;
    const bool still_landed = land_detector_fresh && land_detected_->landed;
    abort_mission(
      still_landed ? "takeoff timeout: land detector masih melaporkan landed" :
      "takeoff timeout: altitude target tidak tercapai dalam batas waktu");
    return;
  }
  if (!takeoff_progress_check_at_.has_value()) {
    takeoff_progress_check_at_ = current_time;
    takeoff_progress_last_down_error_ = down_error;
    return;
  }
  if ((current_time - *takeoff_progress_check_at_).seconds() >= takeoff_stuck_window_s_) {
    const double progress = takeoff_progress_last_down_error_ - down_error;
    if (progress < takeoff_stuck_min_progress_m_) {
      abort_mission("takeoff macet: climb progress di bawah minimum (TAKEOFF_STUCK)");
      return;
    }
    takeoff_progress_check_at_ = current_time;
    takeoff_progress_last_down_error_ = down_error;
  }
}

std::optional<LocalWaypoint> WaypointMission::resolve_waypoint(
  const GlobalWaypoint & waypoint) const
{
  if (!local_position_.has_value() || !local_position_->xy_global || !local_position_->z_global ||
    !std::isfinite(local_position_->ref_lat) || !std::isfinite(local_position_->ref_lon) ||
    !std::isfinite(local_position_->ref_alt))
  {
    return std::nullopt;
  }
  // home_position tidak tersedia (topic tidak dibridge); ref_alt (EKF origin)
  // dipakai sebagai basis altitude REL_HOME. Sudah divalidasi finite di atas.
  const double home_alt =
    waypoint.altitude_frame == AltitudeFrame::kRelativeHome ? local_position_->ref_alt : 0.0;
  auto local = project_global_to_local(
    waypoint, local_position_->ref_lat, local_position_->ref_lon,
    local_position_->ref_alt, home_alt);
  if (use_tf_mini_altitude_ && altitude_ekf_offset_m_.has_value()) {
    // EKF z (ref_alt-relative) terbukti bisa offset besar dari AGL sebenarnya
    // (diverifikasi via tf_mini). Offset konstan hasil kalibrasi takeoff
    // (lihat kTakeoffVertical) dipakai di sini supaya SEMUA waypoint
    // berikutnya -- yang di mission ini semuanya 1m REL_HOME -- position
    // control PX4 (yang jalan di frame EKF-nya sendiri) menuju z yang benar
    // secara fisik, bukan z teoritis dari ref_alt yang sudah terbukti salah.
    local.down_m += *altitude_ekf_offset_m_;
  }
  local.fly_through = waypoint.fly_through;
  return local;
}

bool WaypointMission::waypoint_reached(const LocalWaypoint & waypoint) const
{
  if (!local_position_.has_value() || !local_position_->xy_valid || !local_position_->z_valid ||
    !local_position_->v_xy_valid || !local_position_->v_z_valid)
  {
    return false;
  }
  return horizontal_distance(local_position_->x, local_position_->y, waypoint) <= acceptance_xy_m_ &&
         altitude_reached(waypoint) &&
         speed_3d(local_position_->vx, local_position_->vy, local_position_->vz) <= stopped_speed_mps_;
}

std::optional<double> WaypointMission::tf_mini_altitude_m() const
{
  if (!use_tf_mini_altitude_ || !range_.has_value() ||
    (now() - last_range_at_).seconds() > tf_mini_timeout_s_ ||
    !std::isfinite(range_->range) || range_->range < range_->min_range ||
    range_->range > range_->max_range)
  {
    return std::nullopt;
  }
  return static_cast<double>(range_->range);
}

double WaypointMission::altitude_error_m(const LocalWaypoint & waypoint) const
{
  const auto tf_mini_altitude = tf_mini_altitude_m();
  if (tf_mini_altitude.has_value() && local_position_.has_value()) {
    // waypoint.down_m sudah NED-relative terhadap ref_alt (EKF origin, yang
    // diperlakukan sebagai proksi ground/home level -- lihat resolve_waypoint).
    // AGL target = -down_m langsung; TIDAK perlu ref_alt lagi di sini (formula
    // lama keliru menyisakan suku +ref_alt sehingga error terhitung puluhan
    // meter dan gate altitude tidak pernah terpenuhi -- bug lama, belum pernah
    // kepakai karena use_tf_mini_altitude selalu false sampai sekarang).
    // Kalau sudah terkalibrasi, waypoint.down_m sudah digeser +offset EKF
    // (lihat resolve_waypoint) supaya PX4 position control benar secara
    // fisik -- offset itu harus "dibatalkan" lagi di sini supaya target_agl_m
    // tetap angka AGL murni (mis. 1.0 m), bukan tercampur offset EKF.
    const double target_agl_m = -(waypoint.down_m - altitude_ekf_offset_m_.value_or(0.0));
    return std::abs(target_agl_m - *tf_mini_altitude);
  }
  return std::abs(waypoint.down_m - local_position_->z);
}

bool WaypointMission::altitude_reached(const LocalWaypoint & waypoint) const
{
  if (use_tf_mini_altitude_ && !tf_mini_altitude_m().has_value()) {
    return false;
  }
  return altitude_error_m(waypoint) <= acceptance_z_m_;
}

bool WaypointMission::pass_condition_met(const LocalWaypoint & waypoint) const
{
  if (!local_position_.has_value() || current_index_ + 1U >= waypoints_.size()) {return false;}
  const auto next = resolve_waypoint(waypoints_[current_index_ + 1U]);
  if (!next.has_value()) {return false;}
  const double distance = horizontal_distance(local_position_->x, local_position_->y, waypoint);
  if (distance > fly_through_pass_radius_m_) {return false;}
  const double bearing_to_next = target_heading_for(waypoint, *next);
  const double heading_error = std::abs(wrap_pi(bearing_to_next - local_position_->heading));
  return heading_error <= fly_through_yaw_tolerance_rad_;
}

double WaypointMission::target_heading(const LocalWaypoint & waypoint) const
{
  if (std::isfinite(waypoint.yaw_rad)) {
    return wrap_pi(waypoint.yaw_rad);
  }
  const double delta_north = waypoint.north_m - local_position_->x;
  const double delta_east = waypoint.east_m - local_position_->y;
  if (std::hypot(delta_north, delta_east) < acceptance_xy_m_) {
    return commanded_yaw_;
  }
  return std::atan2(delta_east, delta_north);
}

double WaypointMission::target_heading_for(
  const LocalWaypoint & from, const LocalWaypoint & to) const
{
  const double delta_north = to.north_m - from.north_m;
  const double delta_east = to.east_m - from.east_m;
  if (std::hypot(delta_north, delta_east) < 1e-3) {return commanded_yaw_;}
  return std::atan2(delta_east, delta_north);
}

bool WaypointMission::yaw_rate_settled() const
{
  if (!require_yaw_rate_feedback_) {return true;}
  if (!angular_velocity_.has_value() ||
    (now() - last_angular_velocity_at_).seconds() > angular_velocity_timeout_s_)
  {
    return false;
  }
  return std::abs(static_cast<double>(angular_velocity_->xyz[2])) <= yaw_rate_stopped_rad_s_;
}

double WaypointMission::limited_yaw(const double desired_yaw)
{
  const auto current_time = now();
  double dt = 1.0 / publish_rate_hz_;
  if (last_tick_time_.has_value()) {
    const double measured = (current_time - *last_tick_time_).seconds();
    if (measured > 1e-4 && measured < 1.0) {dt = measured;}
  }
  last_tick_time_ = current_time;

  if (!yaw_initialized_) {
    commanded_yaw_ = local_position_.has_value() && std::isfinite(local_position_->heading) ?
      local_position_->heading : desired_yaw;
    commanded_yaw_rate_rad_s_ = 0.0;
    yaw_initialized_ = true;
  }

  const double error = wrap_pi(desired_yaw - commanded_yaw_);
  const double desired_rate = std::clamp(
    error / std::max(dt, 1e-3), -max_yaw_rate_rad_s_, max_yaw_rate_rad_s_);
  const double max_rate_delta = max_yaw_accel_rad_s2_ * dt;
  const double rate_delta = std::clamp(
    desired_rate - commanded_yaw_rate_rad_s_, -max_rate_delta, max_rate_delta);
  commanded_yaw_rate_rad_s_ = std::clamp(
    commanded_yaw_rate_rad_s_ + rate_delta, -max_yaw_rate_rad_s_, max_yaw_rate_rad_s_);
  yaw_rate_saturated_ = std::abs(error / std::max(dt, 1e-3)) > max_yaw_rate_rad_s_ + 1e-6 ||
    std::abs(rate_delta) >= max_rate_delta - 1e-9;
  commanded_yaw_ = wrap_pi(commanded_yaw_ + commanded_yaw_rate_rad_s_ * dt);
  return commanded_yaw_;
}

void WaypointMission::publish_target(const LocalWaypoint & waypoint)
{
  geometry_msgs::msg::PoseStamped msg{};
  msg.header.stamp = now();
  msg.header.frame_id = "px4_ned";
  msg.pose.position.x = waypoint.north_m;
  msg.pose.position.y = waypoint.east_m;
  msg.pose.position.z = waypoint.down_m;
  const double yaw = limited_yaw(waypoint.yaw_rad);
  msg.pose.orientation.w = std::cos(yaw * 0.5);
  msg.pose.orientation.z = std::sin(yaw * 0.5);
  target_pub_->publish(msg);
  if (local_position_.has_value()) {
    const auto tf_mini_altitude = tf_mini_altitude_m();
    // gate_agl/gate_error = angka yang SEBENARNYA dipakai watchdog/reached
    // (tf_mini kalau use_tf_mini_altitude=true, kalau tidak dari EKF z).
    // ekf_agl/ekf_z dicetak terpisah sebagai referensi silang -- keduanya
    // BISA beda cukup jauh kalau EKF origin/ref_alt sedang tidak sinkron
    // (lihat catatan altitude_error_m).
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Altitude track: target_down=%.3f | gate=%s gate_agl=%.3f gate_error=%.3f | "
      "ekf_z=%.3f ekf_agl=%.3f ekf_error=%.3f ref_alt=%.3f",
      waypoint.down_m,
      use_tf_mini_altitude_ ? "tf_mini" : "ekf_z",
      use_tf_mini_altitude_ && tf_mini_altitude.has_value() ? *tf_mini_altitude : -local_position_->z,
      altitude_error_m(waypoint),
      local_position_->z, -local_position_->z, waypoint.down_m - local_position_->z,
      local_position_->ref_alt);
  }
  std_msgs::msg::UInt32 state{};
  state.data = static_cast<uint32_t>(current_index_ + 1U);
  state_pub_->publish(state);
  publish_feedforward();
}

void WaypointMission::publish_feedforward()
{
  geometry_msgs::msg::TwistStamped msg{};
  msg.header.stamp = now();
  msg.header.frame_id = "px4_ned";
  msg.twist.linear.x = NAN;
  msg.twist.linear.y = NAN;
  msg.twist.linear.z = NAN;
  if (phase_ == Phase::kNavigate && current_index_ < waypoints_.size() &&
    waypoints_[current_index_].fly_through && local_position_.has_value())
  {
    const auto current = resolve_waypoint(waypoints_[current_index_]);
    if (current.has_value()) {
      const double dn = current->north_m - local_position_->x;
      const double de = current->east_m - local_position_->y;
      const double dd = current->down_m - local_position_->z;
      const double distance = std::sqrt(dn * dn + de * de + dd * dd);
      if (distance > 1e-3) {
        const double speed = std::min(fly_through_speed_mps_, distance * publish_rate_hz_);
        msg.twist.linear.x = speed * dn / distance;
        msg.twist.linear.y = speed * de / distance;
        msg.twist.linear.z = speed * dd / distance;
      }
    }
  }
  msg.twist.angular.z = commanded_yaw_rate_rad_s_;
  feedforward_pub_->publish(msg);
}

void WaypointMission::enter_alignment(const LocalWaypoint & waypoint)
{
  segment_yaw_ = target_heading(waypoint);
  alignment_hold_ = LocalWaypoint{
    local_position_->x, local_position_->y, local_position_->z, segment_yaw_, 0.0, false};
  phase_ = Phase::kAlignHeading;
  alignment_started_at_ = now();
  yaw_progress_check_at_ = now();
  yaw_progress_last_abs_error_ = std::abs(wrap_pi(segment_yaw_ - local_position_->heading));
  RCLCPP_INFO(get_logger(), "Align heading ke %.3f rad sebelum WP%zu", segment_yaw_, current_index_ + 1U);
}

void WaypointMission::begin_takeoff(const LocalWaypoint & takeoff_target)
{
  takeoff_target_ = takeoff_target;
  takeoff_started_at_ = now();
  takeoff_progress_check_at_ = now();
  takeoff_progress_last_down_error_ = std::abs(takeoff_target.down_m - local_position_->z);
}

void WaypointMission::tick()
{
  if (mission_aborted_) {
    abort_mission("abort tetap aktif");
    return;
  }
  if (!local_position_.has_value() || current_index_ >= waypoints_.size() ||
    !local_position_->xy_valid || !local_position_->z_valid ||
    !std::isfinite(local_position_->heading))
  {
    return;
  }
  const bool offboard_active = vehicle_status_.has_value() &&
    vehicle_status_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  if (!estimator_healthy()) {
    if (offboard_active || phase_ != Phase::kWaitForOffboard) {
      abort_mission("estimator/global position/failsafe health gate gagal");
    }
    return;
  }
  if (!mission_feasible_) {
    mission_feasible_ = check_mission_feasibility();
    if (!mission_feasible_) {return;}
    // Mission tervalidasi (feasibility+geofence+altitude+jarak home) -- baru
    // sekarang aman untuk supervisor mengirim arm. Keputusan "boleh arm"
    // milik mission node, bukan otomatis begitu OFFBOARD terkonfirmasi.
    std_msgs::msg::Bool arm_msg{};
    arm_msg.data = true;
    arm_request_pub_->publish(arm_msg);
    RCLCPP_INFO(get_logger(), "Mission siap; arm request dikirim ke offboard_supervisor");
  }
  if (local_position_->ref_timestamp != initial_ref_timestamp_ &&
    phase_ != Phase::kWaitForOffboard)
  {
    abort_mission("home position (EKF origin ref) berubah saat mission aktif");
    return;
  }
  check_estimator_resets();
  if (mission_aborted_) {return;}
  if (require_offboard_ && (!vehicle_status_.has_value() ||
    vehicle_status_->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD))
  {
    phase_ = Phase::kWaitForOffboard;
    return;
  }
  const auto current = resolve_waypoint(waypoints_[current_index_]);
  if (!current.has_value()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
      "LLA tidak dapat diproyeksikan: ref global EKF/home altitude belum valid");
    return;
  }
  // Sanity gate altitude cepat (per-tick, tanpa window) -- terpisah dari
  // check_takeoff_watchdog() yang hanya menangkap climb yang TERLALU LAMBAT.
  // Kalau altitude error melompat besar (mis. akibat hold position usang di
  // offboard_supervisor yang stale karena EKF origin reset di antara startup
  // supervisor dan arm -- pernah terverifikasi live menyebabkan vehicle
  // "chase" ke ~11m padahal target 1m), gate ini abort dalam ~1 tick,
  // bukan menunggu window watchdog stuck yang bisa beberapa detik.
  // Pakai altitude_error_m() (bukan z mentah) supaya otomatis tf_mini-aware:
  // sebelum kalibrasi offset EKF, error terhadap ref_alt teoritis memang
  // besar dan itu diharapkan (bukan tanda bahaya) -- tf_mini yang jadi
  // acuan kebenaran, bukan z EKF yang belum tentu sinkron.
  if (std::abs(altitude_error_m(*current)) > max_altitude_error_m_) {
    abort_mission("altitude error melebihi batas sanity (ALTITUDE_ENVELOPE)");
    return;
  }

  if (phase_ == Phase::kWaitForOffboard) {
    initial_yaw_ = local_position_->heading;
    commanded_yaw_ = initial_yaw_;
    commanded_yaw_rate_rad_s_ = 0.0;
    yaw_initialized_ = true;
    begin_takeoff(
      LocalWaypoint{local_position_->x, local_position_->y, current->down_m, initial_yaw_, 0.0, false});
    phase_ = Phase::kTakeoffVertical;
    RCLCPP_INFO(get_logger(), "Takeoff vertikal; yaw awal dikunci %.3f rad", initial_yaw_);
  }

  switch (phase_) {
    case Phase::kTakeoffVertical: {
        if (use_tf_mini_altitude_ && !altitude_ekf_offset_m_.has_value()) {
          // Belum terkalibrasi: naik pakai velocity setpoint Z (posisi Z
          // di-NaN-kan) dipandu langsung oleh tf_mini, supaya PX4 tidak
          // pernah membandingkan terhadap down_m teoritis (yang bergantung
          // pada ref_alt -- terbukti live bisa offset beberapa meter dari
          // ketinggian fisik sebenarnya, menyebabkan PX4 mengira sudah
          // sampai/lewat target dan tidak pernah naik).
          const auto tf_alt = tf_mini_altitude_m();
          const double target_agl = -takeoff_target_.down_m;
          const double agl_error = tf_alt.has_value() ? target_agl - *tf_alt : 0.0;
          const double climb_vel = tf_alt.has_value() ?
            std::clamp(agl_error, -tf_mini_climb_rate_mps_, tf_mini_climb_rate_mps_) : 0.0;
          // Feedforward (berisi velocity Z riil) dipublish SEBELUM target
          // posisi (yang z-nya NaN) supaya begitu offboard_supervisor
          // memproses target NaN, feedforward yang jadi pasangannya sudah
          // fresh -- menghindari window z=NaN tanpa velocity pengganti.
          geometry_msgs::msg::TwistStamped ff_msg{};
          ff_msg.header.stamp = now();
          ff_msg.header.frame_id = "px4_ned";
          ff_msg.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
          ff_msg.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
          ff_msg.twist.linear.z = -climb_vel;
          ff_msg.twist.angular.z = commanded_yaw_rate_rad_s_;
          feedforward_pub_->publish(ff_msg);

          geometry_msgs::msg::PoseStamped pose_msg{};
          pose_msg.header.stamp = now();
          pose_msg.header.frame_id = "px4_ned";
          pose_msg.pose.position.x = takeoff_target_.north_m;
          pose_msg.pose.position.y = takeoff_target_.east_m;
          pose_msg.pose.position.z = std::numeric_limits<double>::quiet_NaN();
          const double yaw = limited_yaw(takeoff_target_.yaw_rad);
          pose_msg.pose.orientation.w = std::cos(yaw * 0.5);
          pose_msg.pose.orientation.z = std::sin(yaw * 0.5);
          target_pub_->publish(pose_msg);
          std_msgs::msg::UInt32 state{};
          state.data = static_cast<uint32_t>(current_index_ + 1U);
          state_pub_->publish(state);

          check_takeoff_watchdog();
          if (mission_aborted_) {break;}

          if (tf_alt.has_value() && std::abs(agl_error) <= acceptance_z_m_) {
            altitude_ekf_offset_m_ = local_position_->z - takeoff_target_.down_m;
            takeoff_target_.down_m += *altitude_ekf_offset_m_;
            RCLCPP_INFO(get_logger(),
              "Altitude terkalibrasi via tf_mini: offset EKF=%.3f m (tf_mini=%.3f target_agl=%.3f)",
              *altitude_ekf_offset_m_, *tf_alt, target_agl);
          }
          break;
        }
        publish_target(takeoff_target_);
        check_takeoff_watchdog();
        if (mission_aborted_) {break;}
        const bool land_detector_fresh = land_detected_.has_value() &&
          (now() - last_land_detected_at_).seconds() <= land_detected_timeout_s_;
        const bool confirmed_airborne = !land_detector_fresh || !land_detected_->landed;
        if (waypoint_reached(takeoff_target_) && confirmed_airborne) {
          takeoff_started_at_.reset();
          takeoff_progress_check_at_.reset();
          enter_alignment(*current);
        }
      }
      break;

    case Phase::kAlignHeading:
      alignment_hold_.yaw_rad = segment_yaw_;
      publish_target(alignment_hold_);
      check_yaw_alignment_watchdog();
      if (mission_aborted_) {break;}
      if (std::abs(wrap_pi(segment_yaw_ - local_position_->heading)) <= yaw_acceptance_rad_ &&
        std::abs(wrap_pi(segment_yaw_ - commanded_yaw_)) <= yaw_acceptance_rad_ &&
        !yaw_rate_saturated_ && yaw_rate_settled())
      {
        phase_ = Phase::kNavigate;
        alignment_started_at_.reset();
        yaw_progress_check_at_.reset();
        RCLCPP_INFO(get_logger(), "Heading aligned; translasi menuju WP%zu", current_index_ + 1U);
      }
      break;

    case Phase::kNavigate: {
        LocalWaypoint target = *current;
        target.yaw_rad = segment_yaw_;
        // Setpoint yang dikirim ke PX4 dilebihkan sedikit ke arah gerak
        // (waypoint_forward_bias_m) supaya trajectory generator PX4 approach
        // dengan momentum yang lebih tegas alih-alih pelan-pelan settle pas
        // di tepi acceptance_xy_m -- reached/hold tetap dievaluasi terhadap
        // titik waypoint asli (target), bukan versi yang dilebihkan ini.
        LocalWaypoint send_target = target;
        send_target.north_m += std::cos(segment_yaw_) * waypoint_forward_bias_m_;
        send_target.east_m += std::sin(segment_yaw_) * waypoint_forward_bias_m_;
        publish_target(send_target);
        if (waypoints_[current_index_].fly_through && pass_condition_met(target)) {
          ++current_index_;
          const auto next = resolve_waypoint(waypoints_[current_index_]);
          if (next.has_value()) {
            segment_yaw_ = target_heading_for(target, *next);
            reached_since_.reset();
            RCLCPP_INFO(get_logger(), "Fly-through: lanjut translasi ke WP%zu", current_index_ + 1U);
          }
        } else if (waypoint_reached(target)) {
          reached_since_ = now();
          phase_ = Phase::kHold;
          RCLCPP_INFO(get_logger(), "WP%zu reached; mulai hold %.2f s",
            current_index_ + 1U, target.hold_s);
        }
      }
      break;

    case Phase::kHold: {
        LocalWaypoint target = *current;
        target.yaw_rad = segment_yaw_;
        publish_target(target);
        if (!waypoint_reached(target)) {
          reached_since_ = now();
        }
        if (reached_since_.has_value() &&
          (now() - *reached_since_).seconds() >= target.hold_s)
        {
          if (current_index_ + 1U < waypoints_.size()) {
            ++current_index_;
            const auto next = resolve_waypoint(waypoints_[current_index_]);
            if (next.has_value()) {enter_alignment(*next);}
          } else {
            phase_ = Phase::kFinished;
          }
        }
      }
      break;

    case Phase::kFinished: {
        LocalWaypoint target = *current;
        target.yaw_rad = segment_yaw_;
        publish_target(target);
        if (!land_requested_) {
          land_requested_ = true;
          std_msgs::msg::Bool land_msg{};
          land_msg.data = true;
          land_request_pub_->publish(land_msg);
          RCLCPP_INFO(get_logger(),
            "WP%zu (terakhir) tercapai; auto-land diminta ke offboard_supervisor",
            current_index_ + 1U);
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "Mission selesai; hold waypoint terakhir dan heartbeat aktif");
      }
      break;

    case Phase::kWaitForOffboard:
      break;
  }
}

}  // namespace waypoint_mission
