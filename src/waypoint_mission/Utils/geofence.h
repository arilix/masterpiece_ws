#ifndef PX4_WAYPOINT_MISSION__GEOFENCE_H_
#define PX4_WAYPOINT_MISSION__GEOFENCE_H_

// Custom geofence check (GAP_IMPLEMENTASI_DAN_ROADMAP.md §8.1). Ini melengkapi,
// bukan menggantikan, geofence native PX4 (failsafe_flags_->geofence_breached)
// yang tetap dipantau saat runtime. Polygon diperiksa di bidang lokal NED yang
// sama dengan waypoint (proyeksi tangent-plane terhadap referensi EKF), bukan
// lat/lon langsung, agar konsisten dengan sisa geometri workspace ini.

#include "waypoint.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace waypoint_mission
{

struct PlanarPoint
{
  double north_m{};
  double east_m{};
};

using PlanarPolygon = std::vector<PlanarPoint>;

struct LatLonPoint
{
  double latitude_deg{};
  double longitude_deg{};
};

inline LatLonPoint parse_latlon_point(const std::string & text)
{
  const auto comma = text.find(',');
  if (comma == std::string::npos) {
    throw std::runtime_error("Titik geofence wajib 'lat,lon': '" + text + "'");
  }
  LatLonPoint point{};
  try {
    point.latitude_deg = std::stod(trim(text.substr(0, comma)));
    point.longitude_deg = std::stod(trim(text.substr(comma + 1)));
  } catch (const std::exception &) {
    throw std::runtime_error("Angka titik geofence tidak valid: '" + text + "'");
  }
  if (!std::isfinite(point.latitude_deg) || point.latitude_deg < -90.0 ||
    point.latitude_deg > 90.0 || !std::isfinite(point.longitude_deg) ||
    point.longitude_deg < -180.0 || point.longitude_deg > 180.0)
  {
    throw std::runtime_error("Range lat/lon geofence tidak valid: '" + text + "'");
  }
  return point;
}

// Proyeksi titik fence ke bidang lokal yang sama dengan waypoint (tangent
// plane terhadap referensi EKF saat ini). Fence harus diproyeksikan ulang
// setiap kali dipakai karena referensi EKF dapat berubah (reset/relokasi).
inline PlanarPoint project_latlon_to_planar(
  const LatLonPoint & point, const double ref_lat_deg, const double ref_lon_deg)
{
  const GlobalWaypoint as_waypoint{
    point.latitude_deg, point.longitude_deg, 0.0, AltitudeFrame::kAmsl, NAN, 0.0};
  const auto local = project_global_to_local(as_waypoint, ref_lat_deg, ref_lon_deg, 0.0, 0.0);
  return PlanarPoint{local.north_m, local.east_m};
}

inline PlanarPolygon project_polygon(
  const std::vector<LatLonPoint> & polygon, const double ref_lat_deg, const double ref_lon_deg)
{
  PlanarPolygon planar;
  planar.reserve(polygon.size());
  for (const auto & point : polygon) {
    planar.push_back(project_latlon_to_planar(point, ref_lat_deg, ref_lon_deg));
  }
  return planar;
}

// Distance dari titik ke segmen AB (untuk margin check).
inline double point_to_segment_distance(
  const PlanarPoint & p, const PlanarPoint & a, const PlanarPoint & b)
{
  const double abx = b.north_m - a.north_m;
  const double aby = b.east_m - a.east_m;
  const double apx = p.north_m - a.north_m;
  const double apy = p.east_m - a.east_m;
  const double ab_len_sq = abx * abx + aby * aby;
  const double t = ab_len_sq > 1e-12 ?
    std::clamp((apx * abx + apy * aby) / ab_len_sq, 0.0, 1.0) : 0.0;
  const double closest_n = a.north_m + t * abx;
  const double closest_e = a.east_m + t * aby;
  return std::hypot(p.north_m - closest_n, p.east_m - closest_e);
}

// Ray casting: true jika titik berada di dalam polygon (boundary dianggap luar
// agar margin check terpisah yang menentukan batas aman).
inline bool point_in_polygon(const PlanarPoint & p, const PlanarPolygon & polygon)
{
  if (polygon.size() < 3U) {return false;}
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1U; i < polygon.size(); j = i++) {
    const auto & pi = polygon[i];
    const auto & pj = polygon[j];
    const bool crosses = ((pi.east_m > p.east_m) != (pj.east_m > p.east_m));
    if (crosses) {
      const double intersect_north = pi.north_m +
        (p.east_m - pi.east_m) * (pj.north_m - pi.north_m) / (pj.east_m - pi.east_m);
      if (p.north_m < intersect_north) {inside = !inside;}
    }
  }
  return inside;
}

inline double distance_to_polygon_boundary(const PlanarPoint & p, const PlanarPolygon & polygon)
{
  double min_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0, j = polygon.size() - 1U; i < polygon.size(); j = i++) {
    min_distance = std::min(min_distance, point_to_segment_distance(p, polygon[j], polygon[i]));
  }
  return min_distance;
}

inline bool segments_intersect(
  const PlanarPoint & a, const PlanarPoint & b, const PlanarPoint & c, const PlanarPoint & d)
{
  const auto cross = [](const PlanarPoint & o, const PlanarPoint & p1, const PlanarPoint & p2) {
      return (p1.north_m - o.north_m) * (p2.east_m - o.east_m) -
             (p1.east_m - o.east_m) * (p2.north_m - o.north_m);
    };
  const double d1 = cross(c, d, a);
  const double d2 = cross(c, d, b);
  const double d3 = cross(a, b, c);
  const double d4 = cross(a, b, d);
  if (((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0))) {return true;}
  return false;
}

// Sample sebuah segmen setiap resolution_m untuk uji point-in-polygon/margin
// pada polygon panjang, bukan hanya endpoint.
inline std::vector<PlanarPoint> sample_segment(
  const PlanarPoint & from, const PlanarPoint & to, const double resolution_m)
{
  const double length = std::hypot(to.north_m - from.north_m, to.east_m - from.east_m);
  const std::size_t steps = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(length / std::max(resolution_m, 0.1))));
  std::vector<PlanarPoint> samples;
  samples.reserve(steps + 1U);
  for (std::size_t i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    samples.push_back(
      PlanarPoint{
        from.north_m + t * (to.north_m - from.north_m),
        from.east_m + t * (to.east_m - from.east_m)});
  }
  return samples;
}

// Uji satu waypoint (titik) terhadap satu inclusion (opsional) dan sejumlah
// exclusion polygon dengan margin. Mengembalikan alasan gagal, atau nullopt
// jika lolos.
inline std::optional<std::string> check_point_against_fence(
  const PlanarPoint & point, const std::optional<PlanarPolygon> & inclusion,
  const std::vector<PlanarPolygon> & exclusions, const double margin_m)
{
  if (inclusion.has_value()) {
    if (!point_in_polygon(point, *inclusion) ||
      distance_to_polygon_boundary(point, *inclusion) < margin_m)
    {
      return "titik di luar inclusion fence atau melanggar margin";
    }
  }
  for (const auto & exclusion : exclusions) {
    if (point_in_polygon(point, exclusion) ||
      distance_to_polygon_boundary(point, exclusion) < margin_m)
    {
      return "titik di dalam exclusion fence atau melanggar margin";
    }
  }
  return std::nullopt;
}

inline std::optional<std::string> check_segment_against_fence(
  const PlanarPoint & from, const PlanarPoint & to,
  const std::optional<PlanarPolygon> & inclusion, const std::vector<PlanarPolygon> & exclusions,
  const double margin_m, const double sample_resolution_m)
{
  for (const auto & sample : sample_segment(from, to, sample_resolution_m)) {
    const auto failure = check_point_against_fence(sample, inclusion, exclusions, margin_m);
    if (failure.has_value()) {return failure;}
  }
  for (const auto & exclusion : exclusions) {
    for (std::size_t i = 0, j = exclusion.size() - 1U; i < exclusion.size(); j = i++) {
      if (segments_intersect(from, to, exclusion[j], exclusion[i])) {
        return "segmen mission memotong boundary exclusion fence";
      }
    }
  }
  return std::nullopt;
}

}  // namespace waypoint_mission

#endif  // PX4_WAYPOINT_MISSION__GEOFENCE_H_
