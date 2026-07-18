#ifndef PX4_WAYPOINT_MISSION__WAYPOINT_H_
#define PX4_WAYPOINT_MISSION__WAYPOINT_H_

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace waypoint_mission
{

// kTerrain diparsing agar operator mendapat error yang jelas, bukan silent
// fallback. Belum ada sumber DEM/datum tervalidasi di workspace ini, sehingga
// mission dengan frame ini WAJIB ditolak saat feasibility check (lihat
// WaypointMission::check_mission_feasibility). Lihat GAP_IMPLEMENTASI_DAN_ROADMAP.md §8.2.
enum class AltitudeFrame {kAmsl, kRelativeHome, kTerrain};

struct GlobalWaypoint
{
  double latitude_deg{};
  double longitude_deg{};
  double altitude_m{};
  AltitudeFrame altitude_frame{AltitudeFrame::kRelativeHome};
  double yaw_rad{NAN};
  double hold_s{};
  // Opsional (field ke-7, default false agar file waypoints.yaml lama tetap
  // valid tanpa perubahan). Hanya bermakna jika hold_s == 0 dan waypoint bukan
  // waypoint terakhir; lihat GAP §5.1. Fitur ini opt-in dan belum divalidasi
  // di flight test -- default seluruh waypoint tetap stop-rotate-go.
  bool fly_through{false};
};

struct LocalWaypoint
{
  double north_m{};
  double east_m{};
  double down_m{};
  double yaw_rad{NAN};
  double hold_s{};
  bool fly_through{false};
};

inline std::string trim(std::string value)
{
  const auto not_space = [](const unsigned char c) {return !std::isspace(c);};
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

inline GlobalWaypoint parse_waypoint(const std::string & text)
{
  std::stringstream stream(text);
  std::string token;
  std::vector<std::string> fields;
  while (std::getline(stream, token, ',')) {
    fields.push_back(trim(token));
  }
  if (fields.size() != 6U && fields.size() != 7U) {
    throw std::runtime_error(
            "Waypoint wajib lat,lon,alt,AMSL|REL_HOME|TERRAIN,yaw,hold[,FLY_THROUGH]: '" +
            text + "'");
  }
  GlobalWaypoint waypoint;
  try {
    waypoint.latitude_deg = std::stod(fields[0]);
    waypoint.longitude_deg = std::stod(fields[1]);
    waypoint.altitude_m = std::stod(fields[2]);
    waypoint.yaw_rad = std::stod(fields[4]);
    waypoint.hold_s = std::stod(fields[5]);
  } catch (const std::exception &) {
    throw std::runtime_error("Angka waypoint tidak valid: '" + text + "'");
  }
  if (fields[3] == "AMSL") {
    waypoint.altitude_frame = AltitudeFrame::kAmsl;
  } else if (fields[3] == "REL_HOME") {
    waypoint.altitude_frame = AltitudeFrame::kRelativeHome;
  } else if (fields[3] == "TERRAIN") {
    waypoint.altitude_frame = AltitudeFrame::kTerrain;
  } else {
    throw std::runtime_error("Altitude frame harus AMSL, REL_HOME, atau TERRAIN: '" + text + "'");
  }
  if (fields.size() == 7U) {
    if (fields[6] == "FLY_THROUGH") {
      waypoint.fly_through = true;
    } else if (!fields[6].empty() && fields[6] != "STOP") {
      throw std::runtime_error("Field ke-7 harus FLY_THROUGH atau STOP: '" + text + "'");
    }
  }
  if (waypoint.fly_through && waypoint.hold_s > 0.0) {
    throw std::runtime_error(
            "FLY_THROUGH tidak boleh dipakai bersama hold_s > 0: '" + text + "'");
  }
  if (!std::isfinite(waypoint.latitude_deg) || waypoint.latitude_deg < -90.0 ||
    waypoint.latitude_deg > 90.0 || !std::isfinite(waypoint.longitude_deg) ||
    waypoint.longitude_deg < -180.0 || waypoint.longitude_deg > 180.0 ||
    !std::isfinite(waypoint.altitude_m) || !std::isfinite(waypoint.hold_s) ||
    waypoint.hold_s < 0.0)
  {
    throw std::runtime_error("Range LLA/hold waypoint tidak valid: '" + text + "'");
  }
  return waypoint;
}

// Sama dengan PX4 MapProjection: spherical Azimuthal Equidistant, R=6371000 m.
inline LocalWaypoint project_global_to_local(
  const GlobalWaypoint & waypoint, const double ref_lat_deg, const double ref_lon_deg,
  const double ref_alt_amsl_m, const double home_alt_amsl_m)
{
  constexpr double earth_radius_m = 6371000.0;
  constexpr double deg_to_rad = M_PI / 180.0;
  const double ref_lat = ref_lat_deg * deg_to_rad;
  const double ref_lon = ref_lon_deg * deg_to_rad;
  const double lat = waypoint.latitude_deg * deg_to_rad;
  const double lon = waypoint.longitude_deg * deg_to_rad;
  const double sin_ref_lat = std::sin(ref_lat);
  const double cos_ref_lat = std::cos(ref_lat);
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double cos_delta_lon = std::cos(lon - ref_lon);
  const double argument = std::clamp(
    sin_ref_lat * sin_lat + cos_ref_lat * cos_lat * cos_delta_lon, -1.0, 1.0);
  const double central_angle = std::acos(argument);
  const double scale = std::abs(central_angle) > 0.0 ?
    central_angle / std::sin(central_angle) : 1.0;
  // kTerrain wajib sudah ditolak oleh feasibility check sebelum fungsi ini
  // dipanggil untuk item tersebut; fallback di sini hanya untuk keamanan
  // aritmatika (bukan pengganti terrain query yang sesungguhnya).
  const double altitude_amsl = waypoint.altitude_frame == AltitudeFrame::kAmsl ?
    waypoint.altitude_m : home_alt_amsl_m + waypoint.altitude_m;
  return LocalWaypoint{
    scale * (cos_ref_lat * sin_lat - sin_ref_lat * cos_lat * cos_delta_lon) * earth_radius_m,
    scale * cos_lat * std::sin(lon - ref_lon) * earth_radius_m,
    -(altitude_amsl - ref_alt_amsl_m), waypoint.yaw_rad, waypoint.hold_s};
}

inline double horizontal_distance(
  const double x, const double y, const LocalWaypoint & waypoint)
{
  return std::hypot(waypoint.north_m - x, waypoint.east_m - y);
}

inline double speed_3d(const double vx, const double vy, const double vz)
{
  return std::sqrt(vx * vx + vy * vy + vz * vz);
}

}  // namespace waypoint_mission

#endif  // PX4_WAYPOINT_MISSION__WAYPOINT_H_
