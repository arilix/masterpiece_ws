#include "waypoint.h"
#include "sha256.h"

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <filesystem>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <utility>

namespace
{

// Lock exclusive non-blocking selama baca-ubah-tulis, agar dua instance
// recorder tidak saling menimpa (GAP §10).
class FileLock
{
public:
  explicit FileLock(const std::string & path)
  {
    fd_ = ::open(path.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw std::runtime_error("Tidak dapat membuka file untuk locking: " + path);
    }
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error(
              "File mission sedang dikunci proses lain (recorder lain berjalan?): " + path);
    }
  }

  ~FileLock()
  {
    if (fd_ >= 0) {
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
    }
  }

  FileLock(const FileLock &) = delete;
  FileLock & operator=(const FileLock &) = delete;

private:
  int fd_{-1};
};

std::string timestamp_tag()
{
  const auto now_time = std::chrono::system_clock::now();
  const std::time_t now_c = std::chrono::system_clock::to_time_t(now_time);
  std::tm tm_buf{};
  gmtime_r(&now_c, &tm_buf);
  std::ostringstream out;
  out << std::put_time(&tm_buf, "%Y%m%dT%H%M%SZ");
  return out.str();
}

void fsync_file(const std::filesystem::path & path)
{
  const int fd = ::open(path.c_str(), O_WRONLY);
  if (fd >= 0) {
    ::fsync(fd);
    ::close(fd);
  }
}

void fsync_directory(const std::filesystem::path & directory)
{
  const int fd = ::open(directory.c_str(), O_RDONLY);
  if (fd >= 0) {
    ::fsync(fd);
    ::close(fd);
  }
}

}  // namespace

class PositionReporter : public rclcpp::Node
{
public:
  PositionReporter()
  : Node("position_reporter"), started_at_(now())
  {
    wp_index_ = static_cast<int>(declare_parameter<int64_t>("wp", 0));
    mission_file_ = declare_parameter("mission_file", std::string{});
    capture_delay_s_ = declare_parameter("capture_delay_seconds", 2.0);
    sample_count_ = static_cast<std::size_t>(declare_parameter<int64_t>("sample_count", 20));
    max_eph_m_ = declare_parameter("max_eph_m", 1.5);
    max_epv_m_ = declare_parameter("max_epv_m", 2.5);
    max_sample_spread_m_ = declare_parameter("max_sample_spread_m", 1.0);
    if (wp_index_ < 0 || wp_index_ > 10) {
      throw std::runtime_error("wp harus 0 (report only) atau 1..10");
    }
    if (wp_index_ > 0 && mission_file_.empty()) {
      throw std::runtime_error("mission_file wajib diisi ketika wp=1..10");
    }
    if (sample_count_ < 3U || max_eph_m_ <= 0.0 || max_epv_m_ <= 0.0 ||
      max_sample_spread_m_ <= 0.0)
    {
      throw std::runtime_error("Parameter quality recorder tidak valid");
    }
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    local_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {local_ = *msg;});
    global_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", qos,
      [this](px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {global_ = *msg;});
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&PositionReporter::report, this));
    if (wp_index_ > 0) {
      RCLCPP_WARN(get_logger(), "Mode record WP%d; posisi valid akan ditulis setelah %.1f s",
        wp_index_, capture_delay_s_);
    }
  }

private:
  void record_waypoint(const double latitude, const double longitude)
  {
    namespace fs = std::filesystem;
    const fs::path configured_path(mission_file_);
    const fs::path target_path = fs::canonical(configured_path);
    // Lock dulu sebelum baca agar baca-ubah-tulis atomik terhadap recorder lain.
    FileLock lock(target_path.string());

    std::ifstream input(target_path, std::ios::binary);
    if (!input) {throw std::runtime_error("Tidak dapat membaca " + target_path.string());}
    std::ostringstream original_stream;
    original_stream << input.rdbuf();
    const std::string original_content = original_stream.str();
    input.close();

    bool schema_version_found = false;
    int schema_version_value = 1;
    std::vector<std::string> lines;
    int found_index = 0;
    bool replaced = false;
    {
      std::istringstream reparse(original_content);
      std::string line;
      while (std::getline(reparse, line)) {
        const std::string stripped = waypoint_mission::trim(line);
        if (stripped.rfind("schema_version:", 0) == 0) {
          schema_version_found = true;
          const auto colon = stripped.find(':');
          try {
            schema_version_value = std::stoi(waypoint_mission::trim(stripped.substr(colon + 1)));
          } catch (const std::exception &) {
            throw std::runtime_error("Nilai schema_version pada file tidak valid");
          }
        }
        if (stripped.rfind("- \"", 0) == 0 && stripped.size() >= 4U && stripped.back() == '"') {
          ++found_index;
          if (found_index == wp_index_) {
            const std::size_t first_quote = line.find('"');
            const std::size_t last_quote = line.rfind('"');
            const auto old = waypoint_mission::parse_waypoint(
              line.substr(first_quote + 1U, last_quote - first_quote - 1U));
            std::ostringstream replacement;
            replacement << line.substr(0, first_quote + 1U) << std::fixed << std::setprecision(8)
                        << latitude << "," << longitude << "," << std::setprecision(3)
                        << old.altitude_m << ","
                        << (old.altitude_frame == waypoint_mission::AltitudeFrame::kAmsl ?
              "AMSL" : (old.altitude_frame == waypoint_mission::AltitudeFrame::kRelativeHome ?
              "REL_HOME" : "TERRAIN")) << ",";
            if (std::isfinite(old.yaw_rad)) {replacement << old.yaw_rad;} else {replacement << "nan";}
            replacement << "," << old.hold_s;
            if (old.fly_through) {replacement << ",FLY_THROUGH";}
            replacement << '"';
            line = replacement.str();
            replaced = true;
          }
        }
        lines.push_back(line);
      }
    }
    if (schema_version_found && schema_version_value != 1) {
      throw std::runtime_error("schema_version file tidak didukung oleh recorder ini (hanya versi 1)");
    }
    if (!replaced) {
      throw std::runtime_error("WP target tidak ditemukan pada file mission aktif");
    }

    std::ostringstream new_content_stream;
    for (const auto & output_line : lines) {new_content_stream << output_line << '\n';}
    const std::string new_content = new_content_stream.str();

    const fs::path temporary = target_path.string() + ".tmp";
    const fs::path backup_directory = target_path.parent_path() / "backup";
    const fs::path backup_latest = backup_directory / (target_path.filename().string() + ".bak");
    const fs::path backup_history =
      backup_directory / (target_path.filename().string() + ".bak." + timestamp_tag());

    {
      std::ofstream output(temporary, std::ios::trunc | std::ios::binary);
      if (!output) {throw std::runtime_error("Tidak dapat menulis file sementara");}
      output << new_content;
      output.flush();
    }
    fsync_file(temporary);
    fs::create_directories(backup_directory);
    fs::copy_file(target_path, backup_history, fs::copy_options::overwrite_existing);
    fs::copy_file(target_path, backup_latest, fs::copy_options::overwrite_existing);
    fs::rename(temporary, target_path);
    fsync_directory(backup_directory);
    fsync_directory(target_path.parent_path());

    recorded_ = true;
    RCLCPP_INFO(get_logger(), "WP%d diperbarui: lat=%.8f lon=%.8f", wp_index_,
      latitude, longitude);
    RCLCPP_INFO(get_logger(), "Backup terbaru: %s", backup_latest.c_str());
    RCLCPP_INFO(get_logger(), "Riwayat revisi: %s", backup_history.c_str());
    RCLCPP_INFO(get_logger(), "Hash sebelum (SHA-256): %s",
      waypoint_mission::Sha256::hash_hex(original_content).c_str());
    RCLCPP_INFO(get_logger(), "Hash sesudah (SHA-256): %s",
      waypoint_mission::Sha256::hash_hex(new_content).c_str());
  }

  void report()
  {
    if (!local_.has_value() || !global_.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Menunggu local/global position PX4");
      return;
    }
    const auto & l = *local_;
    const auto & g = *global_;
    std::ostringstream output;
    output << std::fixed << std::setprecision(8)
           << "LOCAL NED: N=" << l.x << " E=" << l.y << " D=" << l.z
           << " yaw=" << l.heading << " | GLOBAL: lat=" << g.lat << " lon=" << g.lon
           << " AMSL=" << g.alt << " hacc=" << g.eph << " vacc=" << g.epv;
    RCLCPP_INFO(get_logger(), "%s", output.str().c_str());
    if (!recorded_ && wp_index_ > 0 && (now() - started_at_).seconds() >= capture_delay_s_) {
      const bool healthy = g.lat_lon_valid && g.alt_valid && !g.dead_reckoning &&
        l.xy_valid && l.xy_global && !l.dead_reckoning && std::isfinite(g.lat) &&
        std::isfinite(g.lon) && std::isfinite(g.eph) && std::isfinite(g.epv) &&
        g.eph <= max_eph_m_ && g.epv <= max_epv_m_;
      if (!healthy) {
        samples_.clear();
        RCLCPP_WARN(get_logger(), "Sample ditolak: validity/dead-reckoning/eph/epv gate gagal");
        return;
      }
      samples_.emplace_back(g.lat, g.lon);
      RCLCPP_INFO(get_logger(), "Sample recorder %zu/%zu", samples_.size(), sample_count_);
      if (samples_.size() >= sample_count_) {
        double mean_lat = 0.0;
        double mean_lon = 0.0;
        for (const auto & sample : samples_) {mean_lat += sample.first; mean_lon += sample.second;}
        mean_lat /= static_cast<double>(samples_.size());
        mean_lon /= static_cast<double>(samples_.size());
        double max_spread = 0.0;
        for (const auto & sample : samples_) {
          waypoint_mission::GlobalWaypoint point{
            sample.first, sample.second, 0.0,
            waypoint_mission::AltitudeFrame::kAmsl, NAN, 0.0, false};
          const auto offset = waypoint_mission::project_global_to_local(
            point, mean_lat, mean_lon, 0.0, 0.0);
          max_spread = std::max(max_spread, std::hypot(offset.north_m, offset.east_m));
        }
        if (max_spread > max_sample_spread_m_) {
          RCLCPP_ERROR(get_logger(), "Sample spread %.2f m > %.2f m; ulangi sampling",
            max_spread, max_sample_spread_m_);
          samples_.clear();
          return;
        }
        try {record_waypoint(mean_lat, mean_lon);} catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Gagal merekam waypoint: %s", error.what());
          samples_.clear();
        }
      }
    }
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> local_;
  std::optional<px4_msgs::msg::VehicleGlobalPosition> global_;
  rclcpp::Time started_at_;
  int wp_index_{0};
  std::string mission_file_;
  double capture_delay_s_{2.0};
  bool recorded_{false};
  std::size_t sample_count_{20U};
  double max_eph_m_{1.5};
  double max_epv_m_{2.5};
  double max_sample_spread_m_{1.0};
  std::vector<std::pair<double, double>> samples_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionReporter>());
  rclcpp::shutdown();
  return 0;
}
