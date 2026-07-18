#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace
{

struct TopicCheck
{
  const char * topic;
  const char * role;
  bool required;
  bool fail_closed_required;
};

const std::vector<TopicCheck> kTopics{
  {"/fmu/out/vehicle_local_position", "PX4 local position", true, false},
  {"/fmu/out/vehicle_global_position", "PX4 global position", true, false},
  {"/fmu/out/home_position", "PX4 home position", true, false},
  {"/fmu/out/vehicle_status_v1", "PX4 vehicle status", true, false},
  {"/fmu/out/vehicle_command_ack", "PX4 command ACK", true, false},
  {"/fmu/out/failsafe_flags", "PX4 failsafe flags", true, false},
  {"/fmu/out/battery_status", "PX4 battery status", true, false},
  {"/fmu/out/vehicle_land_detected", "PX4 land detector", true, false},
  {
    "/fmu/out/vehicle_angular_velocity",
    "PX4 yaw-rate feedback; optional unless require_yaw_rate_feedback=true",
    false,
    false},
  {
    "/fmu/out/control_allocator_status",
    "PX4 yaw-authority monitor; required when require_control_allocator_status=true",
    false,
    false},
  {"/fmu/in/offboard_control_mode", "PX4 offboard proof-of-life input", true, false},
  {"/fmu/in/trajectory_setpoint", "PX4 trajectory setpoint input", true, false},
  {"/fmu/in/vehicle_command", "PX4 vehicle command input", true, false},
};

struct Args
{
  double timeout_s{6.0};
  bool strict_optional{false};
};

void print_usage(const char * program)
{
  std::cout
    << "Usage: " << program << " [--timeout SECONDS] [--strict-optional] "
    << "[--allow-fail-closed-yaw]\n"
    << "\nAudit topic PX4/ROS 2 yang dibutuhkan mission tanpa publish command apa pun.\n"
    << "  --timeout SECONDS          Timeout discovery topic, detik. Default: 6.0\n"
    << "  --strict-optional          Anggap semua topic optional sebagai gagal bila tidak ada.\n"
    << "  --allow-fail-closed-yaw    Kompatibilitas lama; tidak mengubah hasil audit.\n";
}

Args parse_args(int argc, char ** argv)
{
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg{argv[i]};
    if (arg == "--timeout") {
      if (i + 1 >= argc) {
        throw std::runtime_error("--timeout membutuhkan nilai detik");
      }
      args.timeout_s = std::stod(argv[++i]);
      if (args.timeout_s < 0.0) {
        throw std::runtime_error("--timeout tidak boleh negatif");
      }
    } else if (arg == "--strict-optional") {
      args.strict_optional = true;
    } else if (arg == "--allow-fail-closed-yaw") {
      // Kept for CLI compatibility with the old Python tool.
    } else if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("argumen tidak dikenal: " + arg);
    }
  }
  return args;
}

std::unordered_set<std::string> live_topics(
  const rclcpp::Node::SharedPtr & node, const double timeout_s)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout_s));

  std::unordered_set<std::string> topics;
  do {
    topics.clear();
    for (const auto & [name, types] : node->get_topic_names_and_types()) {
      (void)types;
      if (!name.empty() && name.front() == '/') {
        topics.insert(name);
      }
    }
    if (!topics.empty() || timeout_s == 0.0) {
      break;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while (std::chrono::steady_clock::now() < deadline);
  return topics;
}

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  try {
    args = parse_args(argc, argv);
  } catch (const std::exception & exc) {
    std::cerr << "ERROR: " << exc.what() << '\n';
    print_usage(argv[0]);
    return 2;
  }

  rclcpp::init(argc, argv);
  const auto shutdown = rclcpp::contexts::get_global_default_context();
  auto node = rclcpp::Node::make_shared("audit_topics");

  std::unordered_set<std::string> observed;
  try {
    observed = live_topics(node, args.timeout_s);
  } catch (const std::exception & exc) {
    std::cerr << "ERROR: " << exc.what() << '\n';
    rclcpp::shutdown(shutdown, "audit_topics failed");
    return 2;
  }

  std::vector<TopicCheck> missing_required;
  std::vector<TopicCheck> missing_fail_closed;
  std::vector<TopicCheck> missing_optional;
  for (const auto & check : kTopics) {
    const bool present = observed.find(check.topic) != observed.end();
    const char * status = "OK";
    if (!present) {
      status = check.required ? "MISSING" :
        (check.fail_closed_required ? "FAIL_CLOSED_MISSING" : "OPTIONAL_MISSING");
    }

    std::cout.width(16);
    std::cout << std::left << status << ' ';
    std::cout.width(42);
    std::cout << std::left << check.topic << ' ' << check.role << '\n';

    if (!present && check.required) {
      missing_required.push_back(check);
    } else if (!present && check.fail_closed_required) {
      missing_fail_closed.push_back(check);
    } else if (!present) {
      missing_optional.push_back(check);
    }
  }

  int exit_code = 0;
  if (!missing_required.empty()) {
    std::cerr << "\nFAIL: topic wajib belum tersedia.\n";
    exit_code = 1;
  } else if (args.strict_optional && !missing_optional.empty()) {
    std::cerr << "\nFAIL: topic optional diminta strict tetapi belum tersedia.\n";
    exit_code = 1;
  } else if (!missing_fail_closed.empty() || !missing_optional.empty()) {
    std::cout << "\nPASS dengan catatan: topic optional belum tersedia.\n";
  } else {
    std::cout << "\nPASS: semua topic yang diaudit tersedia.\n";
  }

  rclcpp::shutdown(shutdown, "audit_topics complete");
  return exit_code;
}
