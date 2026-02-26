#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
enum class PlannerState
{
  EXPLORE,
  RECOVERY_HOLD,
};

enum class SourceKind
{
  NONE,
  TARE,
  HOLD_LAST,
};

struct SourceSlot
{
  geometry_msgs::msg::PointStamped latest{};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  bool has_msg{false};
  bool was_fresh{false};
};

struct RobotRuntime
{
  PlannerState state{PlannerState::RECOVERY_HOLD};
  SourceKind active_source{SourceKind::NONE};
  rclcpp::Time state_changed_at{0, 0, RCL_ROS_TIME};

  SourceSlot tare{};
  rclcpp::Time odom_stamp{0, 0, RCL_ROS_TIME};
  bool has_odom{false};

  std::optional<geometry_msgs::msg::PointStamped> last_output{};
  std::string last_mode{"RECOVERY_HOLD"};

  std::unordered_map<std::string, std::uint64_t> transition_counts{};
  std::unordered_map<std::string, std::uint64_t> stale_drop_counts{{"tare", 0}};
  std::uint64_t recovery_entries{0};
  std::uint64_t recovery_exits{0};
  std::uint64_t source_publish_counts{0};
  std::uint64_t hold_publish_counts{0};
  std::uint64_t source_drop_no_output{0};

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr tare_sub{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub{};
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub{};
};

bool is_finite_waypoint(const geometry_msgs::msg::PointStamped & msg)
{
  return std::isfinite(msg.point.x) && std::isfinite(msg.point.y) && std::isfinite(msg.point.z);
}

std::string to_state_string(const PlannerState state)
{
  switch (state) {
    case PlannerState::EXPLORE:
      return "EXPLORE";
    case PlannerState::RECOVERY_HOLD:
      return "RECOVERY_HOLD";
  }
  return "RECOVERY_HOLD";
}

std::string to_source_string(const SourceKind source)
{
  switch (source) {
    case SourceKind::TARE:
      return "tare";
    case SourceKind::HOLD_LAST:
      return "hold_last";
    case SourceKind::NONE:
      break;
  }
  return "none";
}
}  // namespace

class MTareBehaviorExecutiveCpp : public rclcpp::Node
{
public:
  MTareBehaviorExecutiveCpp()
  : Node("mtare_behavior_executive_cpp")
  {
    this->declare_parameter<std::vector<std::string>>("namespaces", {"robot_a", "robot_b"});
    this->declare_parameter<std::string>("tare_input_suffix", "/way_point_tare");
    this->declare_parameter<std::string>("odom_input_suffix", "/odom/nav");
    this->declare_parameter<std::string>("waypoint_output_suffix", "/way_point_coord");
    this->declare_parameter<std::string>("planner_mode_output_suffix", "/planner_mode");
    this->declare_parameter<double>("source_timeout_sec", 2.0);
    this->declare_parameter<double>("output_rate_hz", 8.0);
    this->declare_parameter<bool>("hold_last_output", true);
    this->declare_parameter<bool>("stamp_now", true);
    this->declare_parameter<bool>("enable_hysteresis_guard", false);
    this->declare_parameter<bool>("enable_switch_lock_guard", false);
    this->declare_parameter<bool>("enable_stale_guard", false);
    this->declare_parameter<int>("hysteresis_required_ticks", 2);
    this->declare_parameter<double>("switch_lock_sec", 1.0);
    this->declare_parameter<double>("summary_log_period_sec", 10.0);

    namespaces_ = this->get_parameter("namespaces").as_string_array();
    tare_input_suffix_ = this->get_parameter("tare_input_suffix").as_string();
    odom_input_suffix_ = this->get_parameter("odom_input_suffix").as_string();
    waypoint_output_suffix_ = this->get_parameter("waypoint_output_suffix").as_string();
    planner_mode_output_suffix_ = this->get_parameter("planner_mode_output_suffix").as_string();
    source_timeout_sec_ = std::max(0.0, this->get_parameter("source_timeout_sec").as_double());
    output_rate_hz_ = std::max(1.0, this->get_parameter("output_rate_hz").as_double());
    hold_last_output_ = this->get_parameter("hold_last_output").as_bool();
    stamp_now_ = this->get_parameter("stamp_now").as_bool();
    summary_log_period_sec_ = std::max(1.0, this->get_parameter("summary_log_period_sec").as_double());

    for (const auto & ns : namespaces_) {
      RobotRuntime rt;
      rt.state_changed_at = this->now();
      rt.state = PlannerState::RECOVERY_HOLD;
      rt.active_source = SourceKind::NONE;

      rt.tare_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        topic(ns, tare_input_suffix_),
        10,
        [this, ns](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          this->on_tare_waypoint_msg(ns, *msg);
        });
      rt.odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        topic(ns, odom_input_suffix_),
        10,
        [this, ns](const nav_msgs::msg::Odometry::SharedPtr) {
          auto it = robots_.find(ns);
          if (it == robots_.end()) {
            return;
          }
          it->second.has_odom = true;
          it->second.odom_stamp = this->now();
        });
      rt.waypoint_pub =
        this->create_publisher<geometry_msgs::msg::PointStamped>(topic(ns, waypoint_output_suffix_), 10);
      rt.mode_pub = this->create_publisher<std_msgs::msg::String>(topic(ns, planner_mode_output_suffix_), 10);

      robots_.insert_or_assign(ns, std::move(rt));
    }

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / output_rate_hz_),
      [this]() { this->on_tick(); });

    RCLCPP_INFO(
      this->get_logger(),
      "mtare_behavior_executive_cpp started | namespaces=%zu timeout=%.2fs rate=%.1fHz mode=2-state(EXPLORE/RECOVERY)",
      namespaces_.size(),
      source_timeout_sec_,
      output_rate_hz_);
  }

private:
  std::string topic(const std::string & ns, const std::string & suffix) const
  {
    return "/" + ns + suffix;
  }

  void on_tare_waypoint_msg(const std::string & ns, const geometry_msgs::msg::PointStamped & msg)
  {
    if (!is_finite_waypoint(msg)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "[%s] dropping non-finite tare waypoint",
        ns.c_str());
      return;
    }
    auto it = robots_.find(ns);
    if (it == robots_.end()) {
      return;
    }
    auto & rt = it->second;
    rt.tare.latest = msg;
    rt.tare.stamp = this->now();
    rt.tare.has_msg = true;
  }

  bool source_is_fresh(const SourceSlot & slot, const rclcpp::Time & now) const
  {
    if (!slot.has_msg) {
      return false;
    }
    if (source_timeout_sec_ <= 0.0) {
      return true;
    }
    return (now - slot.stamp).seconds() <= source_timeout_sec_;
  }

  void note_stale_transition(
    RobotRuntime & rt,
    const std::string & ns,
    SourceSlot & slot,
    const bool fresh)
  {
    if (slot.has_msg && slot.was_fresh && !fresh) {
      rt.stale_drop_counts["tare"] += 1;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "[%s] tare source stale (drop_count=%lu)",
        ns.c_str(),
        static_cast<unsigned long>(rt.stale_drop_counts["tare"]));
    }
    slot.was_fresh = fresh;
  }

  PlannerState desired_state_for(const bool tare_fresh) const
  {
    return tare_fresh ? PlannerState::EXPLORE : PlannerState::RECOVERY_HOLD;
  }

  std::pair<std::optional<geometry_msgs::msg::PointStamped>, SourceKind> select_output(
    const RobotRuntime & rt,
    const bool tare_fresh) const
  {
    if (rt.state == PlannerState::EXPLORE && tare_fresh) {
      return {rt.tare.latest, SourceKind::TARE};
    }
    if (hold_last_output_ && rt.last_output.has_value()) {
      return {rt.last_output.value(), SourceKind::HOLD_LAST};
    }
    return {std::nullopt, SourceKind::NONE};
  }

  void update_state(
    const std::string & ns,
    RobotRuntime & rt,
    const PlannerState desired,
    const rclcpp::Time & now)
  {
    if (desired == rt.state) {
      return;
    }

    const std::string from = to_state_string(rt.state);
    const std::string to = to_state_string(desired);
    rt.transition_counts[from + "->" + to] += 1;
    if (desired == PlannerState::RECOVERY_HOLD) {
      rt.recovery_entries += 1;
    }
    if (rt.state == PlannerState::RECOVERY_HOLD && desired != PlannerState::RECOVERY_HOLD) {
      rt.recovery_exits += 1;
    }
    rt.state = desired;
    rt.state_changed_at = now;

    RCLCPP_INFO(this->get_logger(), "[%s] transition %s -> %s", ns.c_str(), from.c_str(), to.c_str());
  }

  void maybe_log_summary(const rclcpp::Time & now)
  {
    if ((now - last_summary_at_).seconds() < summary_log_period_sec_) {
      return;
    }
    last_summary_at_ = now;
    for (const auto & ns : namespaces_) {
      auto it = robots_.find(ns);
      if (it == robots_.end()) {
        continue;
      }
      const auto & rt = it->second;
      const auto stale_tare = rt.stale_drop_counts.find("tare");
      RCLCPP_INFO(
        this->get_logger(),
        "[%s] state=%s source=%s transitions=%zu stale[tare=%lu] "
        "recovery[enter=%lu exit=%lu] outputs[src=%lu hold=%lu dropped=%lu]",
        ns.c_str(),
        to_state_string(rt.state).c_str(),
        to_source_string(rt.active_source).c_str(),
        rt.transition_counts.size(),
        stale_tare == rt.stale_drop_counts.end() ? 0UL : static_cast<unsigned long>(stale_tare->second),
        static_cast<unsigned long>(rt.recovery_entries),
        static_cast<unsigned long>(rt.recovery_exits),
        static_cast<unsigned long>(rt.source_publish_counts),
        static_cast<unsigned long>(rt.hold_publish_counts),
        static_cast<unsigned long>(rt.source_drop_no_output));
    }
  }

  void on_tick()
  {
    const auto now = this->now();
    for (const auto & ns : namespaces_) {
      auto it = robots_.find(ns);
      if (it == robots_.end()) {
        continue;
      }
      auto & rt = it->second;

      const bool tare_fresh = source_is_fresh(rt.tare, now);
      note_stale_transition(rt, ns, rt.tare, tare_fresh);
      update_state(ns, rt, desired_state_for(tare_fresh), now);

      auto selected = select_output(rt, tare_fresh);
      if (selected.first.has_value()) {
        auto msg = selected.first.value();
        if (stamp_now_) {
          msg.header.stamp = now;
        }
        rt.waypoint_pub->publish(msg);
        rt.last_output = msg;
        rt.active_source = selected.second;
        if (selected.second == SourceKind::HOLD_LAST) {
          rt.hold_publish_counts += 1;
        } else {
          rt.source_publish_counts += 1;
        }
      } else {
        rt.active_source = SourceKind::NONE;
        rt.source_drop_no_output += 1;
      }

      const std::string mode = to_state_string(rt.state);
      if (mode != rt.last_mode) {
        RCLCPP_INFO(
          this->get_logger(),
          "[%s] mode=%s source=%s",
          ns.c_str(),
          mode.c_str(),
          to_source_string(rt.active_source).c_str());
        rt.last_mode = mode;
      }
      std_msgs::msg::String mode_msg{};
      mode_msg.data = mode;
      rt.mode_pub->publish(mode_msg);
    }
    maybe_log_summary(now);
  }

  std::vector<std::string> namespaces_{};
  std::string tare_input_suffix_{};
  std::string odom_input_suffix_{};
  std::string waypoint_output_suffix_{};
  std::string planner_mode_output_suffix_{};
  double source_timeout_sec_{2.0};
  double output_rate_hz_{8.0};
  bool hold_last_output_{true};
  bool stamp_now_{true};
  double summary_log_period_sec_{10.0};

  std::unordered_map<std::string, RobotRuntime> robots_{};
  rclcpp::TimerBase::SharedPtr timer_{};
  rclcpp::Time last_summary_at_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MTareBehaviorExecutiveCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
