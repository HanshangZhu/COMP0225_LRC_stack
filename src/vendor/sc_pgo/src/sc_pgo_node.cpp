// sc_pgo_node.cpp — Scan Context Pose Graph Optimization for FAST-LIO2
// Ported from gisbi-kim/SC-A-LOAM (ROS1) to ROS2 rclcpp
// Architecture: multi-threaded — PG construction, SC loop detect, ICP verify, iSAM2 optimize, viz

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "sc_pgo/Scancontext.h"

#include <thread>
#include <mutex>
#include <queue>
#include <optional>
#include <cmath>

using PointType = pcl::PointXYZI;

// ── Pose6D helper ──────────────────────────────────────────────────
struct Pose6D {
  double x, y, z, roll, pitch, yaw;
};

static gtsam::Pose3 toGTSAM(const Pose6D& p) {
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw),
                      gtsam::Point3(p.x, p.y, p.z));
}

static Pose6D fromOdom(const nav_msgs::msg::Odometry& msg) {
  auto& pos = msg.pose.pose.position;
  auto& ori = msg.pose.pose.orientation;
  tf2::Quaternion q(ori.x, ori.y, ori.z, ori.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return {pos.x, pos.y, pos.z, roll, pitch, yaw};
}

static Pose6D diffPose(const Pose6D& a, const Pose6D& b) {
  Eigen::Affine3f T_a = pcl::getTransformation(a.x, a.y, a.z, a.roll, a.pitch, a.yaw);
  Eigen::Affine3f T_b = pcl::getTransformation(b.x, b.y, b.z, b.roll, b.pitch, b.yaw);
  Eigen::Affine3f delta;
  delta.matrix() = T_a.matrix().inverse() * T_b.matrix();
  float dx, dy, dz, dr, dp, dyw;
  pcl::getTranslationAndEulerAngles(delta, dx, dy, dz, dr, dp, dyw);
  return {std::abs(dx), std::abs(dy), std::abs(dz), std::abs(dr), std::abs(dp), std::abs(dyw)};
}

static pcl::PointCloud<PointType>::Ptr transformCloud(
    const pcl::PointCloud<PointType>::Ptr& in, const Pose6D& tf) {
  auto out = std::make_shared<pcl::PointCloud<PointType>>();
  Eigen::Affine3f T = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
  pcl::transformPointCloud(*in, *out, T);
  return out;
}

// ══════════════════════════════════════════════════════════════════
class ScPgoNode : public rclcpp::Node {
public:
  ScPgoNode() : Node("sc_pgo_node") {
    // ── Parameters ──
    declare_parameter("keyframe_meter_gap", 1.0);
    declare_parameter("keyframe_deg_gap", 15.0);
    declare_parameter("sc_max_radius", 20.0);
    declare_parameter("sc_dist_threshold", 0.3);
    declare_parameter("sc_exclude_recent", 25);
    declare_parameter("sc_candidates_num", 10);
    declare_parameter("sc_num_rings", 20);
    declare_parameter("sc_num_sectors", 60);
    declare_parameter("icp_max_correspondence_dist", 2.0);
    declare_parameter("icp_fitness_threshold", 0.3);
    declare_parameter("icp_max_iterations", 100);
    declare_parameter("filter_size", 0.4);
    declare_parameter("map_viz_filter_size", 0.4);
    declare_parameter("frame_id", std::string("camera_init"));
    declare_parameter("child_frame_id", std::string("aft_pgo"));

    kf_meter_gap_ = get_parameter("keyframe_meter_gap").as_double();
    kf_rad_gap_ = get_parameter("keyframe_deg_gap").as_double() * M_PI / 180.0;
    icp_max_corr_ = get_parameter("icp_max_correspondence_dist").as_double();
    icp_fitness_thresh_ = get_parameter("icp_fitness_threshold").as_double();
    icp_max_iters_ = get_parameter("icp_max_iterations").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    child_frame_id_ = get_parameter("child_frame_id").as_string();

    // Scan Context config
    sc::SCConfig sc_cfg;
    sc_cfg.max_radius = get_parameter("sc_max_radius").as_double();
    sc_cfg.dist_threshold = get_parameter("sc_dist_threshold").as_double();
    sc_cfg.exclude_recent = get_parameter("sc_exclude_recent").as_int();
    sc_cfg.candidates_num = get_parameter("sc_candidates_num").as_int();
    sc_cfg.num_rings = get_parameter("sc_num_rings").as_int();
    sc_cfg.num_sectors = get_parameter("sc_num_sectors").as_int();
    sc_manager_ = sc::SCManager(sc_cfg);

    // Voxel filters
    float fs = static_cast<float>(get_parameter("filter_size").as_double());
    ds_sc_.setLeafSize(fs, fs, fs);
    ds_icp_.setLeafSize(fs, fs, fs);
    float mfs = static_cast<float>(get_parameter("map_viz_filter_size").as_double());
    ds_map_.setLeafSize(mfs, mfs, mfs);

    // GTSAM iSAM2
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(params);
    initNoises();

    // ── Subscribers ──
    // Odom: just store latest (Point-LIO publishes at ~3250 Hz, no need to queue)
    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    auto cloud_qos = rclcpp::SensorDataQoS().keep_last(100);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/aft_mapped_to_init", odom_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(odom_mtx_);
          latest_odom_ = msg;
        });
    // Cloud: when a cloud arrives, pair it with latest odom and push to work queue
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", cloud_qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
          nav_msgs::msg::Odometry::SharedPtr odom_msg;
          {
            std::lock_guard<std::mutex> lk(odom_mtx_);
            odom_msg = latest_odom_;
          }
          if (!odom_msg) return;  // no odom yet
          std::lock_guard<std::mutex> lk(buf_mtx_);
          paired_buf_.push({odom_msg, cloud_msg});
        });

    // ── Publishers ──
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/corrected_odom", 100);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/corrected_path", 100);
    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_cloud", 100);
    pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_map", 10);
    pub_loops_ = create_publisher<visualization_msgs::msg::MarkerArray>("/loop_markers", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ── Worker threads ──
    running_ = true;
    th_pg_ = std::thread(&ScPgoNode::processPG, this);
    th_lcd_ = std::thread(&ScPgoNode::processLCD, this);
    th_icp_ = std::thread(&ScPgoNode::processICP, this);
    th_isam_ = std::thread(&ScPgoNode::processISAM, this);
    th_viz_ = std::thread(&ScPgoNode::processViz, this);

    RCLCPP_INFO(get_logger(), "SC-PGO node started (kf_gap=%.1fm/%.0f°, sc_thresh=%.2f, sc_radius=%.0fm)",
                kf_meter_gap_, kf_rad_gap_ * 180.0 / M_PI, sc_cfg.dist_threshold, sc_cfg.max_radius);
  }

  ~ScPgoNode() {
    running_ = false;
    if (th_pg_.joinable()) th_pg_.join();
    if (th_lcd_.joinable()) th_lcd_.join();
    if (th_icp_.joinable()) th_icp_.join();
    if (th_isam_.joinable()) th_isam_.join();
    if (th_viz_.joinable()) th_viz_.join();
  }

private:
  // ── Noise models ─────────────────────────────────────────────────
  void initNoises() {
    gtsam::Vector6 prior_noise;
    prior_noise << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    prior_noise_model_ = gtsam::noiseModel::Diagonal::Variances(prior_noise);

    gtsam::Vector6 odom_noise;
    odom_noise << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odom_noise_model_ = gtsam::noiseModel::Diagonal::Variances(odom_noise);

    gtsam::Vector6 loop_noise;
    loop_noise << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
    loop_noise_model_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(loop_noise));
  }

  // ── Thread: Pose Graph Construction ──────────────────────────────
  void processPG() {
    double trans_accum = 1e6;  // force first frame as keyframe
    double rot_accum = 1e6;
    Pose6D prev_pose{0, 0, 0, 0, 0, 0};
    Pose6D curr_pose{0, 0, 0, 0, 0, 0};

    while (running_) {
      // Grab next paired (odom, cloud) from the work queue
      std::pair<nav_msgs::msg::Odometry::SharedPtr,
                sensor_msgs::msg::PointCloud2::SharedPtr> pair;
      {
        std::lock_guard<std::mutex> lk(buf_mtx_);
        if (paired_buf_.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }
        pair = paired_buf_.front();
        paired_buf_.pop();
      }

      auto& odom_msg = pair.first;
      auto& cloud_msg = pair.second;

      // Parse data
      auto thisKF = std::make_shared<pcl::PointCloud<PointType>>();
      pcl::fromROSMsg(*cloud_msg, *thisKF);
      curr_pose = fromOdom(*odom_msg);
      double ts = rclcpp::Time(odom_msg->header.stamp).seconds();

      // Keyframe gating
      Pose6D delta = diffPose(prev_pose, curr_pose);
      double d_trans = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
      trans_accum += d_trans;
      rot_accum += (delta.roll + delta.pitch + delta.yaw);
      prev_pose = curr_pose;

      if (trans_accum < kf_meter_gap_ && rot_accum < kf_rad_gap_)
        continue;

      trans_accum = 0.0;
      rot_accum = 0.0;

      // Downsample for SC
      auto kfDS = std::make_shared<pcl::PointCloud<PointType>>();
      ds_sc_.setInputCloud(thisKF);
      ds_sc_.filter(*kfDS);

      // Store keyframe
      {
        std::lock_guard<std::mutex> lk(kf_mtx_);
        kf_clouds_.push_back(kfDS);
        kf_poses_.push_back(curr_pose);
        kf_poses_corrected_.push_back(curr_pose);
        kf_times_.push_back(ts);
        sc_manager_.addKeyframe(*kfDS);
      }

      int curr_idx = static_cast<int>(kf_poses_.size()) - 1;
      int prev_idx = curr_idx - 1;

      if (curr_idx == 0) {
        // Prior factor
        auto pose0 = toGTSAM(curr_pose);
        std::lock_guard<std::mutex> lk(pg_mtx_);
        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose0, prior_noise_model_));
        initial_.insert(0, pose0);
        graph_made_ = true;
        RCLCPP_INFO(get_logger(), "PG: prior node 0 added");
      } else {
        auto poseFrom = toGTSAM(kf_poses_[prev_idx]);
        auto poseTo = toGTSAM(kf_poses_[curr_idx]);
        auto relPose = poseFrom.between(poseTo);

        std::lock_guard<std::mutex> lk(pg_mtx_);
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_idx, curr_idx, relPose, odom_noise_model_));
        initial_.insert(curr_idx, poseTo);

        if (curr_idx % 10 == 0)
          RCLCPP_INFO(get_logger(), "PG: odom node %d added (%.1f, %.1f, %.1f)",
                      curr_idx, curr_pose.x, curr_pose.y, curr_pose.z);
      }
    }
  }

  // ── Thread: Scan Context Loop Detection ──────────────────────────
  void processLCD() {
    while (running_) {
      std::this_thread::sleep_for(std::chrono::seconds(1));

      int n;
      {
        std::lock_guard<std::mutex> lk(kf_mtx_);
        n = sc_manager_.numKeyframes();
      }
      if (n < sc_manager_.config().exclude_recent + 2) continue;

      std::pair<int, int> result;
      {
        std::lock_guard<std::mutex> lk(kf_mtx_);
        result = sc_manager_.detectLoopClosure();
      }

      if (result.first != -1) {
        int loop_idx = result.first;
        int curr_idx = n - 1;
        RCLCPP_INFO(get_logger(), "SC loop detected: %d ↔ %d", loop_idx, curr_idx);

        std::lock_guard<std::mutex> lk(icp_mtx_);
        icp_queue_.push({loop_idx, curr_idx});
      }
    }
  }

  // ── Thread: ICP Verification ─────────────────────────────────────
  void processICP() {
    while (running_) {
      std::pair<int, int> pair;
      {
        std::lock_guard<std::mutex> lk(icp_mtx_);
        if (icp_queue_.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
        pair = icp_queue_.front();
        icp_queue_.pop();
      }

      auto rel_pose = doICP(pair.first, pair.second);
      if (rel_pose) {
        std::lock_guard<std::mutex> lk(pg_mtx_);
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            pair.first, pair.second, *rel_pose, loop_noise_model_));

        loop_pairs_.push_back(pair);
        RCLCPP_WARN(get_logger(), "✅ Loop closure added: %d ↔ %d", pair.first, pair.second);
      }
    }
  }

  // ── ICP matching between two keyframes ───────────────────────────
  std::optional<gtsam::Pose3> doICP(int loop_idx, int curr_idx) {
    constexpr int submap_size = 25;
    auto curr_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    auto target_cloud = std::make_shared<pcl::PointCloud<PointType>>();

    {
      std::lock_guard<std::mutex> lk(kf_mtx_);
      // Current keyframe in the loop candidate's frame
      *curr_cloud = *transformCloud(kf_clouds_[curr_idx], kf_poses_corrected_[loop_idx]);

      // Build local submap around loop candidate
      for (int i = -submap_size; i <= submap_size; i++) {
        int idx = loop_idx + i;
        if (idx < 0 || idx >= static_cast<int>(kf_clouds_.size())) continue;
        *target_cloud += *transformCloud(kf_clouds_[idx], kf_poses_corrected_[loop_idx]);
      }
    }

    // Downsample
    auto curr_ds = std::make_shared<pcl::PointCloud<PointType>>();
    auto target_ds = std::make_shared<pcl::PointCloud<PointType>>();
    ds_icp_.setInputCloud(curr_cloud);
    ds_icp_.filter(*curr_ds);
    ds_icp_.setInputCloud(target_cloud);
    ds_icp_.filter(*target_ds);

    if (curr_ds->empty() || target_ds->empty()) return std::nullopt;

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(icp_max_corr_);
    icp.setMaximumIterations(icp_max_iters_);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    icp.setInputSource(curr_ds);
    icp.setInputTarget(target_ds);
    auto result = std::make_shared<pcl::PointCloud<PointType>>();
    icp.align(*result);

    if (!icp.hasConverged() || icp.getFitnessScore() > icp_fitness_thresh_) {
      RCLCPP_INFO(get_logger(), "ICP rejected: %d ↔ %d (fitness=%.3f > %.3f)",
                  loop_idx, curr_idx, icp.getFitnessScore(), icp_fitness_thresh_);
      return std::nullopt;
    }

    RCLCPP_INFO(get_logger(), "ICP passed: %d ↔ %d (fitness=%.3f)", loop_idx, curr_idx, icp.getFitnessScore());

    Eigen::Affine3f correction;
    correction.matrix() = icp.getFinalTransformation();
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(correction, x, y, z, roll, pitch, yaw);

    auto poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    auto poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
    return poseFrom.between(poseTo);
  }

  // ── Thread: iSAM2 Optimization ───────────────────────────────────
  void processISAM() {
    while (running_) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      if (!graph_made_) continue;

      {
        std::lock_guard<std::mutex> lk(pg_mtx_);
        isam_->update(graph_, initial_);
        isam_->update();
        graph_.resize(0);
        initial_.clear();
        estimate_ = isam_->calculateEstimate();
      }

      // Update corrected poses
      {
        std::lock_guard<std::mutex> lk(kf_mtx_);
        for (int i = 0; i < static_cast<int>(kf_poses_corrected_.size()); i++) {
          auto p = estimate_.at<gtsam::Pose3>(i);
          kf_poses_corrected_[i] = {
              p.translation().x(), p.translation().y(), p.translation().z(),
              p.rotation().roll(), p.rotation().pitch(), p.rotation().yaw()};
        }
        recent_idx_ = static_cast<int>(kf_poses_corrected_.size()) - 1;
      }
    }
  }

  // ── Thread: Visualization ────────────────────────────────────────
  void processViz() {
    while (running_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (recent_idx_ < 1) continue;

      std::lock_guard<std::mutex> lk(kf_mtx_);

      // Publish corrected odom (latest keyframe)
      const auto& p = kf_poses_corrected_[recent_idx_];
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.frame_id = frame_id_;
      odom_msg.child_frame_id = child_frame_id_;
      odom_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(kf_times_[recent_idx_] * 1e9));
      odom_msg.pose.pose.position.x = p.x;
      odom_msg.pose.pose.position.y = p.y;
      odom_msg.pose.pose.position.z = p.z;
      tf2::Quaternion q;
      q.setRPY(p.roll, p.pitch, p.yaw);
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();
      pub_odom_->publish(odom_msg);

      // TF broadcast
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = odom_msg.header;
      tf_msg.child_frame_id = child_frame_id_;
      tf_msg.transform.translation.x = p.x;
      tf_msg.transform.translation.y = p.y;
      tf_msg.transform.translation.z = p.z;
      tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);

      // Publish corrected path
      nav_msgs::msg::Path path_msg;
      path_msg.header.frame_id = frame_id_;
      path_msg.header.stamp = odom_msg.header.stamp;
      for (int i = 0; i <= recent_idx_; i++) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id_;
        ps.header.stamp = rclcpp::Time(static_cast<int64_t>(kf_times_[i] * 1e9));
        const auto& pp = kf_poses_corrected_[i];
        ps.pose.position.x = pp.x;
        ps.pose.position.y = pp.y;
        ps.pose.position.z = pp.z;
        tf2::Quaternion qq;
        qq.setRPY(pp.roll, pp.pitch, pp.yaw);
        ps.pose.orientation.x = qq.x();
        ps.pose.orientation.y = qq.y();
        ps.pose.orientation.z = qq.z();
        ps.pose.orientation.w = qq.w();
        path_msg.poses.push_back(ps);
      }
      pub_path_->publish(path_msg);

      // Publish latest corrected cloud
      if (recent_idx_ < static_cast<int>(kf_clouds_.size())) {
        auto corrected = transformCloud(kf_clouds_[recent_idx_], kf_poses_corrected_[recent_idx_]);
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*corrected, cloud_msg);
        cloud_msg.header.frame_id = frame_id_;
        cloud_msg.header.stamp = odom_msg.header.stamp;
        pub_cloud_->publish(cloud_msg);
      }

      // Publish full corrected map (all keyframes, throttled every 5 keyframes)
      if (recent_idx_ > 0 && recent_idx_ % 5 == 0) {
        pcl::PointCloud<PointType>::Ptr full_map(new pcl::PointCloud<PointType>());
        int n = std::min(static_cast<int>(kf_clouds_.size()),
                         static_cast<int>(kf_poses_corrected_.size()));
        for (int i = 0; i < n; i++) {
          *full_map += *transformCloud(kf_clouds_[i], kf_poses_corrected_[i]);
        }
        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
        ds_map_.setInputCloud(full_map);
        ds_map_.filter(*filtered);

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*filtered, map_msg);
        map_msg.header.frame_id = frame_id_;
        map_msg.header.stamp = odom_msg.header.stamp;
        pub_map_->publish(map_msg);
        RCLCPP_INFO(get_logger(), "Published corrected map (%d pts from %d keyframes)",
                    (int)filtered->size(), n);
      }

      // Publish loop closure markers
      if (!loop_pairs_.empty()) {
        visualization_msgs::msg::MarkerArray markers;
        int id = 0;
        for (const auto& [li, ci] : loop_pairs_) {
          if (li >= static_cast<int>(kf_poses_corrected_.size()) ||
              ci >= static_cast<int>(kf_poses_corrected_.size()))
            continue;
          visualization_msgs::msg::Marker m;
          m.header.frame_id = frame_id_;
          m.header.stamp = now();
          m.ns = "loops";
          m.id = id++;
          m.type = visualization_msgs::msg::Marker::LINE_STRIP;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.scale.x = 0.1;
          m.color.r = 0.0;
          m.color.g = 1.0;
          m.color.b = 0.0;
          m.color.a = 1.0;
          geometry_msgs::msg::Point pt1, pt2;
          pt1.x = kf_poses_corrected_[li].x;
          pt1.y = kf_poses_corrected_[li].y;
          pt1.z = kf_poses_corrected_[li].z;
          pt2.x = kf_poses_corrected_[ci].x;
          pt2.y = kf_poses_corrected_[ci].y;
          pt2.z = kf_poses_corrected_[ci].z;
          m.points.push_back(pt1);
          m.points.push_back(pt2);
          markers.markers.push_back(m);
        }
        pub_loops_->publish(markers);
      }
    }
  }

  // ── State ────────────────────────────────────────────────────────
  sc::SCManager sc_manager_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_;
  gtsam::Values estimate_;
  bool graph_made_ = false;

  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_model_;
  gtsam::noiseModel::Diagonal::shared_ptr odom_noise_model_;
  gtsam::noiseModel::Base::shared_ptr loop_noise_model_;

  // Keyframe storage
  std::vector<pcl::PointCloud<PointType>::Ptr> kf_clouds_;
  std::vector<Pose6D> kf_poses_;           // raw odom poses
  std::vector<Pose6D> kf_poses_corrected_; // PGO-corrected
  std::vector<double> kf_times_;
  std::vector<std::pair<int, int>> loop_pairs_;
  int recent_idx_ = 0;

  // Buffers
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::queue<std::pair<nav_msgs::msg::Odometry::SharedPtr,
                       sensor_msgs::msg::PointCloud2::SharedPtr>> paired_buf_;
  std::queue<std::pair<int, int>> icp_queue_;

  // Mutexes
  std::mutex odom_mtx_;   // protects latest_odom_
  std::mutex buf_mtx_;    // protects paired_buf_
  std::mutex kf_mtx_;
  std::mutex pg_mtx_;
  std::mutex icp_mtx_;

  // Filters
  pcl::VoxelGrid<PointType> ds_sc_, ds_icp_, ds_map_;

  // Parameters
  double kf_meter_gap_;
  double kf_rad_gap_;
  double icp_max_corr_;
  double icp_fitness_thresh_;
  int icp_max_iters_;
  std::string frame_id_, child_frame_id_;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_loops_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Threads
  std::atomic<bool> running_{false};
  std::thread th_pg_, th_lcd_, th_icp_, th_isam_, th_viz_;
};

// ══════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScPgoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
