#pragma once
// Scan Context — LiDAR global descriptor for place recognition
// Based on: Kim & Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition within 3D Point Cloud Map" (IROS 2018)
// Simplified header-only implementation for the sc_pgo ROS2 package.

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>

namespace sc {

using PointType = pcl::PointXYZI;
using SCType = Eigen::MatrixXd;       // rings × sectors
using RingKeyType = Eigen::VectorXd;  // 1D ring key for fast lookup

struct SCConfig {
  int    num_rings       = 20;
  int    num_sectors     = 60;
  double max_radius      = 20.0;  // metres — use lower for indoor (20), higher for outdoor (80)
  double dist_threshold  = 0.3;   // scan context distance threshold (lower = stricter)
  int    exclude_recent  = 25;    // skip recent N keyframes when searching for loops
  int    candidates_num  = 10;    // number of nearest ring-key candidates to evaluate
};

class SCManager {
public:
  SCManager() = default;
  explicit SCManager(const SCConfig& cfg) : cfg_(cfg) {}

  void setConfig(const SCConfig& cfg) { cfg_ = cfg; }

  // ── Build descriptor from a point cloud ──────────────────────────
  SCType makeScanContext(const pcl::PointCloud<PointType>& cloud) const {
    SCType sc = SCType::Zero(cfg_.num_rings, cfg_.num_sectors);
    Eigen::MatrixXd counter = Eigen::MatrixXd::Zero(cfg_.num_rings, cfg_.num_sectors);

    const double ring_step = cfg_.max_radius / cfg_.num_rings;
    const double sector_step = 2.0 * M_PI / cfg_.num_sectors;

    for (const auto& pt : cloud.points) {
      double r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
      if (r < 0.01 || r > cfg_.max_radius) continue;

      double theta = std::atan2(pt.y, pt.x) + M_PI;  // [0, 2π)
      int ri = std::min(static_cast<int>(r / ring_step), cfg_.num_rings - 1);
      int si = std::min(static_cast<int>(theta / sector_step), cfg_.num_sectors - 1);

      // max height encoding
      double z = pt.z;
      if (z > sc(ri, si)) {
        sc(ri, si) = z;
      }
    }
    return sc;
  }

  // ── Ring key: mean of each ring (for fast KNN search) ────────────
  RingKeyType makeRingKey(const SCType& sc) const {
    RingKeyType rk(cfg_.num_rings);
    for (int i = 0; i < cfg_.num_rings; i++) {
      rk(i) = sc.row(i).mean();
    }
    return rk;
  }

  // ── Add a keyframe ───────────────────────────────────────────────
  void addKeyframe(const pcl::PointCloud<PointType>& cloud) {
    SCType sc = makeScanContext(cloud);
    RingKeyType rk = makeRingKey(sc);
    sc_database_.push_back(sc);
    rk_database_.push_back(rk);
  }

  // ── Detect loop: returns (matched_idx, yaw_diff_in_columns), or (-1, 0) ─
  std::pair<int, int> detectLoopClosure() const {
    if (static_cast<int>(sc_database_.size()) <= cfg_.exclude_recent + 1)
      return {-1, 0};

    const SCType& query_sc = sc_database_.back();
    const RingKeyType& query_rk = rk_database_.back();
    int n_candidates = static_cast<int>(rk_database_.size()) - cfg_.exclude_recent - 1;
    if (n_candidates <= 0) return {-1, 0};

    // Step 1: Find top-K nearest ring-key candidates (L2 distance)
    std::vector<std::pair<double, int>> rk_dists;
    rk_dists.reserve(n_candidates);
    for (int i = 0; i < n_candidates; i++) {
      double d = (query_rk - rk_database_[i]).norm();
      rk_dists.emplace_back(d, i);
    }
    std::partial_sort(rk_dists.begin(),
                      rk_dists.begin() + std::min(cfg_.candidates_num, n_candidates),
                      rk_dists.end());

    // Step 2: For each candidate, compute SC distance with column shift (yaw invariance)
    double best_dist = 1e9;
    int best_idx = -1;
    int best_shift = 0;

    int k = std::min(cfg_.candidates_num, n_candidates);
    for (int c = 0; c < k; c++) {
      int cand_idx = rk_dists[c].second;
      auto [dist, shift] = scDistance(query_sc, sc_database_[cand_idx]);
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = cand_idx;
        best_shift = shift;
      }
    }

    if (best_dist < cfg_.dist_threshold) {
      return {best_idx, best_shift};
    }
    return {-1, 0};
  }

  int numKeyframes() const { return static_cast<int>(sc_database_.size()); }
  const SCConfig& config() const { return cfg_; }

private:
  // ── SC distance with column-wise circular shift ──────────────────
  //    Returns (min_distance, best_shift)
  std::pair<double, int> scDistance(const SCType& a, const SCType& b) const {
    double best_dist = 1e9;
    int best_shift = 0;

    for (int shift = 0; shift < cfg_.num_sectors; shift++) {
      double dist = columnShiftedDistance(a, b, shift);
      if (dist < best_dist) {
        best_dist = dist;
        best_shift = shift;
      }
    }
    return {best_dist, best_shift};
  }

  // ── Cosine distance between column-shifted descriptors ───────────
  double columnShiftedDistance(const SCType& a, const SCType& b, int shift) const {
    double sum_dist = 0.0;
    int valid_cols = 0;

    for (int si = 0; si < cfg_.num_sectors; si++) {
      int si_shifted = (si + shift) % cfg_.num_sectors;
      Eigen::VectorXd col_a = a.col(si);
      Eigen::VectorXd col_b = b.col(si_shifted);

      // Skip if either column is all zeros
      double norm_a = col_a.norm();
      double norm_b = col_b.norm();
      if (norm_a < 1e-6 || norm_b < 1e-6) continue;

      double cosine_sim = col_a.dot(col_b) / (norm_a * norm_b);
      cosine_sim = std::clamp(cosine_sim, -1.0, 1.0);
      sum_dist += (1.0 - cosine_sim);
      valid_cols++;
    }

    if (valid_cols == 0) return 1e9;
    return sum_dist / valid_cols;
  }

  SCConfig cfg_;
  std::vector<SCType> sc_database_;
  std::vector<RingKeyType> rk_database_;
};

}  // namespace sc
