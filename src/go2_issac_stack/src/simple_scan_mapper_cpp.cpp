#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{
double yaw_from_quat(double x, double y, double z, double w)
{
  const double siny = 2.0 * (w * z + x * y);
  const double cosy = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny, cosy);
}
}  // namespace

class SimpleScanMapperCpp : public rclcpp::Node
{
public:
  SimpleScanMapperCpp()
  : Node("simple_scan_mapper")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom/nav");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    map_frame_ = declare_parameter<std::string>("map_frame", "world");
    lidar_offset_x_ = declare_parameter<double>("lidar_offset_x", 0.0);
    lidar_offset_y_ = declare_parameter<double>("lidar_offset_y", 0.0);

    resolution_ = declare_parameter<double>("resolution", 0.10);
    width_ = declare_parameter<int>("width", 400);
    height_ = declare_parameter<int>("height", 400);
    origin_x_ = declare_parameter<double>("origin_x", -20.0);
    origin_y_ = declare_parameter<double>("origin_y", -20.0);

    max_range_ = std::max(0.1, declare_parameter<double>("max_range", 12.0));
    max_clear_distance_ = std::max(0.0, declare_parameter<double>("max_clear_distance", 4.0));
    clear_on_nohit_ = declare_parameter<bool>("clear_on_nohit", false);
    update_rate_ = std::max(0.5, declare_parameter<double>("update_rate", 4.0));
    startup_delay_ = std::max(0.0, declare_parameter<double>("startup_delay", 4.0));
    max_scan_odom_dt_ = std::max(0.0, declare_parameter<double>("max_scan_odom_dt", 0.0));
    odom_history_sec_ = std::max(0.5, declare_parameter<double>("odom_history_sec", 2.0));

    hit_increment_ = std::max(1, static_cast<int>(declare_parameter("hit_increment", 3)));
    miss_decrement_ = std::max(1, static_cast<int>(declare_parameter("miss_decrement", 1)));
    score_min_ = static_cast<int>(declare_parameter("score_min", -20));
    score_max_ = static_cast<int>(declare_parameter("score_max", 20));
    if (score_min_ >= score_max_) {
      score_min_ = -20;
      score_max_ = 20;
    }
    occupied_score_threshold_ = static_cast<int>(declare_parameter("occupied_score_threshold", 3));
    free_score_threshold_ = static_cast<int>(declare_parameter("free_score_threshold", -3));
    if (free_score_threshold_ >= occupied_score_threshold_) {
      free_score_threshold_ = -3;
      occupied_score_threshold_ = 3;
    }

    const int64_t n_cells = static_cast<int64_t>(width_) * static_cast<int64_t>(height_);
    grid_.assign(static_cast<size_t>(n_cells), -1);
    scores_.assign(static_cast<size_t>(n_cells), 0);
    observed_.assign(static_cast<size_t>(n_cells), false);

    const auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, scan_qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_ = msg;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_ = msg;

        rclcpp::Time stamp(msg->header.stamp);
        int64_t stamp_ns = stamp.nanoseconds();
        if (stamp_ns <= 0) {
          stamp_ns = now().nanoseconds();
        }
        odom_hist_.emplace_back(stamp_ns, msg);

        const int64_t prune_before = stamp_ns - static_cast<int64_t>(odom_history_sec_ * 1e9);
        while (!odom_hist_.empty() && odom_hist_.front().first < prune_before) {
          odom_hist_.pop_front();
        }
      });

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 1);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_),
      std::bind(&SimpleScanMapperCpp::update, this));

    RCLCPP_INFO(
      get_logger(),
      "Simple scan mapper (C++) started | scan=%s odom=%s map=%s size=%dx%d res=%.2f "
      "lidar_offset=(%.3f,%.3f) score=[%d,%d] hit=%d miss=%d odom_history=%.1fs",
      scan_topic_.c_str(), odom_topic_.c_str(), map_topic_.c_str(), width_, height_, resolution_,
      lidar_offset_x_, lidar_offset_y_, score_min_, score_max_, hit_increment_, miss_decrement_,
      odom_history_sec_);
  }

private:
  using OdomMsg = nav_msgs::msg::Odometry;
  using ScanMsg = sensor_msgs::msg::LaserScan;

  std::optional<std::pair<int, int>> world_to_grid(double x, double y) const
  {
    const int gx = static_cast<int>((x - origin_x_) / resolution_);
    const int gy = static_cast<int>((y - origin_y_) / resolution_);
    if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) {
      return std::nullopt;
    }
    return std::make_pair(gx, gy);
  }

  inline size_t idx(int gx, int gy) const
  {
    return static_cast<size_t>(gy * width_ + gx);
  }

  void apply_evidence(int gx, int gy, int delta)
  {
    const size_t i = idx(gx, gy);
    observed_[i] = true;
    int score = scores_[i] + delta;
    score = std::clamp(score, score_min_, score_max_);
    scores_[i] = score;
  }

  // Carve free space with Bresenham and skip endpoint so occupied mark can be
  // applied separately.
  void raytrace_free(int x0, int y0, int x1, int y1)
  {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int x = x0;
    int y = y0;
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;

    if (dx > dy) {
      double err = static_cast<double>(dx) / 2.0;
      while (x != x1) {
        apply_evidence(x, y, -miss_decrement_);
        err -= static_cast<double>(dy);
        if (err < 0.0) {
          y += sy;
          err += static_cast<double>(dx);
        }
        x += sx;
      }
      return;
    }

    double err = static_cast<double>(dy) / 2.0;
    while (y != y1) {
      apply_evidence(x, y, -miss_decrement_);
      err -= static_cast<double>(dx);
      if (err < 0.0) {
        x += sx;
        err += static_cast<double>(dy);
      }
      y += sy;
    }
  }

  std::pair<OdomMsg::SharedPtr, double> select_odom_for_scan(const ScanMsg::SharedPtr & scan) const
  {
    if (odom_hist_.empty()) {
      return {last_odom_, std::numeric_limits<double>::infinity()};
    }

    const int64_t scan_t = rclcpp::Time(scan->header.stamp).nanoseconds();
    if (scan_t <= 0) {
      return {last_odom_, 0.0};
    }

    OdomMsg::SharedPtr best_msg;
    double best_dt = std::numeric_limits<double>::infinity();
    for (const auto & [odom_t, odom_msg] : odom_hist_) {
      const double dt = std::abs(static_cast<double>(scan_t - odom_t)) / 1e9;
      if (dt < best_dt) {
        best_dt = dt;
        best_msg = odom_msg;
      }
    }
    return {best_msg, best_dt};
  }

  void publish_map(const builtin_interfaces::msg::Time & stamp)
  {
    const size_t n = scores_.size();
    for (size_t i = 0; i < n; ++i) {
      if (!observed_[i]) {
        grid_[i] = -1;
      } else if (scores_[i] >= occupied_score_threshold_) {
        grid_[i] = 100;
      } else if (scores_[i] <= free_score_threshold_) {
        grid_[i] = 0;
      } else {
        grid_[i] = -1;
      }
    }

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.info.resolution = static_cast<float>(resolution_);
    msg.info.width = static_cast<uint32_t>(width_);
    msg.info.height = static_cast<uint32_t>(height_);
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.orientation.w = 1.0;
    msg.data = grid_;
    map_pub_->publish(msg);
  }

  void update()
  {
    if (!last_scan_ || !last_odom_) {
      return;
    }

    const auto now_t = now();
    if (!start_time_) {
      start_time_ = now_t;
    }
    if ((now_t - *start_time_).seconds() < startup_delay_) {
      return;
    }

    const auto scan = last_scan_;
    const auto [odom, matched_dt] = select_odom_for_scan(scan);
    if (!odom) {
      return;
    }
    last_match_dt_ = matched_dt;

    if (max_scan_odom_dt_ > 0.0 && std::isfinite(matched_dt) && matched_dt > max_scan_odom_dt_) {
      if ((now_t.nanoseconds() - last_sync_warn_ns_) > static_cast<int64_t>(2e9)) {
        RCLCPP_WARN(
          get_logger(), "scan/odom desync: dt=%.3fs > %.3fs; skipping map update",
          matched_dt, max_scan_odom_dt_);
        last_sync_warn_ns_ = now_t.nanoseconds();
      }
      return;
    }

    const double rx = odom->pose.pose.position.x;
    const double ry = odom->pose.pose.position.y;
    const double yaw = yaw_from_quat(
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z,
      odom->pose.pose.orientation.w);

    // Ray origin from physical lidar mount, not base center.
    const double sx = rx + (std::cos(yaw) * lidar_offset_x_ - std::sin(yaw) * lidar_offset_y_);
    const double sy = ry + (std::sin(yaw) * lidar_offset_x_ + std::cos(yaw) * lidar_offset_y_);

    const auto origin_cell = world_to_grid(sx, sy);
    if (!origin_cell) {
      return;
    }

    double angle = static_cast<double>(scan->angle_min);
    const double inc = static_cast<double>(scan->angle_increment);
    const double range_min = static_cast<double>(scan->range_min);

    for (const float rng_f : scan->ranges) {
      const double rng = static_cast<double>(rng_f);
      const bool finite = std::isfinite(rng);
      if (finite && rng < range_min) {
        angle += inc;
        continue;
      }

      const double dist = finite ? std::min(rng, max_range_) : max_range_;
      const bool has_hit = finite && (rng < max_range_ * 0.99);
      const double world_bearing = yaw + angle;
      const double ex = sx + dist * std::cos(world_bearing);
      const double ey = sy + dist * std::sin(world_bearing);
      const auto end_cell = world_to_grid(ex, ey);

      const double clear_dist =
        (max_clear_distance_ <= 0.0) ? dist : std::min(dist, max_clear_distance_);
      const double cex = sx + clear_dist * std::cos(world_bearing);
      const double cey = sy + clear_dist * std::sin(world_bearing);
      const auto clear_end_cell = world_to_grid(cex, cey);

      angle += inc;
      if (!clear_end_cell) {
        continue;
      }

      if (has_hit || clear_on_nohit_) {
        raytrace_free(origin_cell->first, origin_cell->second, clear_end_cell->first, clear_end_cell->second);
      }

      if (has_hit && end_cell) {
        apply_evidence(end_cell->first, end_cell->second, hit_increment_);
      } else if (finite && clear_on_nohit_ && end_cell) {
        apply_evidence(end_cell->first, end_cell->second, -miss_decrement_);
      }
    }

    publish_map(scan->header.stamp);

    if (last_summary_ns_ == 0 || (now_t.nanoseconds() - last_summary_ns_) > static_cast<int64_t>(10e9)) {
      last_summary_ns_ = now_t.nanoseconds();
      int free_n = 0;
      int occ_n = 0;
      for (const int8_t v : grid_) {
        if (v == 0) {
          ++free_n;
        } else if (v == 100) {
          ++occ_n;
        }
      }
      const int unknown_n = static_cast<int>(grid_.size()) - free_n - occ_n;
      RCLCPP_INFO(
        get_logger(), "MAP step: free=%d occ=%d unknown=%d matched_dt=%.3fs",
        free_n, occ_n, unknown_n, last_match_dt_);
    }
  }

  std::string scan_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string map_frame_;

  double lidar_offset_x_{0.0};
  double lidar_offset_y_{0.0};
  double resolution_{0.10};
  int width_{400};
  int height_{400};
  double origin_x_{-20.0};
  double origin_y_{-20.0};

  double max_range_{12.0};
  double max_clear_distance_{4.0};
  bool clear_on_nohit_{false};
  double update_rate_{4.0};
  double startup_delay_{4.0};
  double max_scan_odom_dt_{0.0};
  double odom_history_sec_{2.0};

  int hit_increment_{3};
  int miss_decrement_{1};
  int score_min_{-20};
  int score_max_{20};
  int occupied_score_threshold_{3};
  int free_score_threshold_{-3};

  std::vector<int8_t> grid_;
  std::vector<int> scores_;
  std::vector<bool> observed_;

  ScanMsg::SharedPtr last_scan_;
  OdomMsg::SharedPtr last_odom_;
  std::deque<std::pair<int64_t, OdomMsg::SharedPtr>> odom_hist_;

  std::optional<rclcpp::Time> start_time_;
  int64_t last_sync_warn_ns_{0};
  int64_t last_summary_ns_{0};
  double last_match_dt_{0.0};

  rclcpp::Subscription<ScanMsg>::SharedPtr scan_sub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleScanMapperCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
