/**
 * reconstruct_scene.cpp — Offline LiDAR+IMU scene reconstruction
 *
 * Reads raw /utlidar/cloud + /utlidar/imu from a Go2 ROS2 bag,
 * applies body-frame transforms, IMU dead-reckoning, ICP scan matching,
 * loop closure detection, pose graph optimization (Ceres), then outputs
 * a PLY point cloud and trajectory.
 *
 * Usage: ./reconstruct_scene <bag_path> [--subsample N]
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <unordered_map>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <yaml-cpp/yaml.h>

// ROS2 bag reading
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

// ═══════════════════════════════════════════════════════════════
// Calibration
// ═══════════════════════════════════════════════════════════════

struct Calibration {
    double acc_bias_x = 0, acc_bias_y = 0, acc_bias_z = 0;
    double ang_bias_x = 0, ang_bias_y = 0, ang_bias_z = 0;
    double ang_z2x_proj = 0.15, ang_z2y_proj = -0.28;
};

Calibration load_calib() {
    Calibration c;
    std::vector<std::string> dirs = {
        std::string(getenv("HOME")) + "/COMP0225_LRC_stack",
        std::string(getenv("HOME")) + "/Desktop"
    };
    for (auto& d : dirs) {
        std::string path = d + "/imu_calib_data.yaml";
        if (std::ifstream(path).good()) {
            YAML::Node cfg = YAML::LoadFile(path);
            if (cfg["acc_bias_x"]) c.acc_bias_x = cfg["acc_bias_x"].as<double>();
            if (cfg["acc_bias_y"]) c.acc_bias_y = cfg["acc_bias_y"].as<double>();
            if (cfg["acc_bias_z"]) c.acc_bias_z = cfg["acc_bias_z"].as<double>();
            if (cfg["ang_bias_x"]) c.ang_bias_x = cfg["ang_bias_x"].as<double>();
            if (cfg["ang_bias_y"]) c.ang_bias_y = cfg["ang_bias_y"].as<double>();
            if (cfg["ang_bias_z"]) c.ang_bias_z = cfg["ang_bias_z"].as<double>();
            if (cfg["ang_z2x_proj"]) c.ang_z2x_proj = cfg["ang_z2x_proj"].as<double>();
            if (cfg["ang_z2y_proj"]) c.ang_z2y_proj = cfg["ang_z2y_proj"].as<double>();
            std::cout << "  Loaded calibration from " << path << std::endl;
            break;
        }
    }
    return c;
}

// ═══════════════════════════════════════════════════════════════
// Data structures
// ═══════════════════════════════════════════════════════════════

struct ImuReading {
    int64_t stamp_ns;
    Eigen::Vector3d gyro, acc;
};

struct Scan {
    int64_t stamp_ns;
    CloudT::Ptr cloud;  // body frame
};

// ═══════════════════════════════════════════════════════════════
// Transforms (matching transform_everything.py)
// ═══════════════════════════════════════════════════════════════

const double THETA = 15.1 * M_PI / 180.0;
const double CAM_OFFSET = 0.046825;

Eigen::Matrix3d body2cloud_rotation() {
    // Euler(0, 2.878..., 0) — same as transform_everything
    double pitch = 2.87820258505555555556;
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    return R;
}

CloudT::Ptr transform_cloud(const sensor_msgs::msg::PointCloud2& msg,
                             const Eigen::Matrix3d& R_body_cloud) {
    auto cloud = std::make_shared<CloudT>();
    int n = msg.width * msg.height;
    if (n == 0) return cloud;

    // Find field offsets
    int xo = -1, yo = -1, zo = -1;
    for (auto& f : msg.fields) {
        if (f.name == "x") xo = f.offset;
        if (f.name == "y") yo = f.offset;
        if (f.name == "z") zo = f.offset;
    }
    if (xo < 0) return cloud;

    cloud->reserve(n);
    const uint8_t* raw = msg.data.data();
    int ps = msg.point_step;

    // Body-box filter bounds
    double bx0 = -0.7, bx1 = -0.1;
    double by0 = -0.3, by1 = 0.3;
    double bz0 = -0.6 - CAM_OFFSET, bz1 = 0.0 - CAM_OFFSET;

    for (int i = 0; i < n; i++) {
        const uint8_t* p = raw + i * ps;
        float fx, fy, fz;
        memcpy(&fx, p + xo, 4);
        memcpy(&fy, p + yo, 4);
        memcpy(&fz, p + zo, 4);

        if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz))
            continue;

        Eigen::Vector3d pt(fx, fy, fz);
        pt = R_body_cloud * pt;
        pt.z() -= CAM_OFFSET;

        // Body-box filter
        if (pt.x() > bx0 && pt.x() < bx1 &&
            pt.y() > by0 && pt.y() < by1 &&
            pt.z() > bz0 && pt.z() < bz1)
            continue;

        cloud->push_back(PointT(pt.x(), pt.y(), pt.z()));
    }
    return cloud;
}

ImuReading transform_imu(const sensor_msgs::msg::Imu& msg, const Calibration& cal) {
    ImuReading r;
    r.stamp_ns = msg.header.stamp.sec * (int64_t)1e9 + msg.header.stamp.nanosec;

    double gx = msg.angular_velocity.x;
    double gy = -msg.angular_velocity.y;
    double gz = -msg.angular_velocity.z;
    double gx2 = cos(THETA)*gx - sin(THETA)*gz;
    double gy2 = gy;
    double gz2 = sin(THETA)*gx + cos(THETA)*gz;
    gx2 -= cal.ang_bias_x; gy2 -= cal.ang_bias_y; gz2 -= cal.ang_bias_z;
    gx2 += cal.ang_z2x_proj * gz2;
    gy2 += cal.ang_z2y_proj * gz2;

    double ax = msg.linear_acceleration.x;
    double ay = -msg.linear_acceleration.y;
    double az = -msg.linear_acceleration.z;
    double ax2 = cos(THETA)*ax - sin(THETA)*az;
    double ay2 = ay;
    double az2 = sin(THETA)*ax + cos(THETA)*az;
    ax2 -= cal.acc_bias_x; ay2 -= cal.acc_bias_y; az2 -= cal.acc_bias_z;

    r.gyro = {gx2, gy2, gz2};
    r.acc = {ax2, ay2, az2};
    return r;
}

// ═══════════════════════════════════════════════════════════════
// 1. Read bag
// ═══════════════════════════════════════════════════════════════

void read_bag(const std::string& path, int subsample,
              const Calibration& cal, const Eigen::Matrix3d& R_bc,
              std::vector<Scan>& scans, std::vector<ImuReading>& imus) {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions opts;
    opts.uri = path;
    opts.storage_id = "sqlite3";
    reader.open(opts);

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_ser;
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;

    int cloud_count = 0;
    while (reader.has_next()) {
        auto msg = reader.read_next();
        if (msg->topic_name == "/utlidar/cloud") {
            cloud_count++;
            if (cloud_count % subsample != 0) continue;

            sensor_msgs::msg::PointCloud2 cloud_msg;
            rclcpp::SerializedMessage smsg(*msg->serialized_data);
            cloud_ser.deserialize_message(&smsg, &cloud_msg);

            auto cloud = transform_cloud(cloud_msg, R_bc);
            if (cloud->size() > 50) {
                int64_t stamp = cloud_msg.header.stamp.sec * (int64_t)1e9 +
                                cloud_msg.header.stamp.nanosec;
                scans.push_back({stamp, cloud});
            }
        } else if (msg->topic_name == "/utlidar/imu") {
            sensor_msgs::msg::Imu imu_msg;
            rclcpp::SerializedMessage smsg(*msg->serialized_data);
            imu_ser.deserialize_message(&smsg, &imu_msg);
            imus.push_back(transform_imu(imu_msg, cal));
        }
    }

    std::sort(scans.begin(), scans.end(),
              [](const Scan& a, const Scan& b){ return a.stamp_ns < b.stamp_ns; });
    std::sort(imus.begin(), imus.end(),
              [](const ImuReading& a, const ImuReading& b){ return a.stamp_ns < b.stamp_ns; });

    std::cout << "  " << cloud_count << " total clouds → " << scans.size()
              << " after subsample=" << subsample << std::endl;
    std::cout << "  " << imus.size() << " IMU readings" << std::endl;
}

// ═══════════════════════════════════════════════════════════════
// 2. IMU dead reckoning
// ═══════════════════════════════════════════════════════════════

std::vector<Eigen::Matrix4d> imu_dead_reckon(
        const std::vector<Scan>& scans, const std::vector<ImuReading>& imus) {
    int n = scans.size();
    std::vector<Eigen::Matrix4d> poses(n);
    poses[0] = Eigen::Matrix4d::Identity();

    size_t imu_idx = 0;
    for (int i = 1; i < n; i++) {
        int64_t t0 = scans[i-1].stamp_ns, t1 = scans[i].stamp_ns;
        Eigen::Vector3d omega_sum = Eigen::Vector3d::Zero();

        while (imu_idx < imus.size() && imus[imu_idx].stamp_ns < t0) imu_idx++;
        size_t j = imu_idx;
        int64_t prev_t = t0;
        while (j < imus.size() && imus[j].stamp_ns <= t1) {
            double dt = (imus[j].stamp_ns - prev_t) * 1e-9;
            if (dt > 0 && dt < 0.1)
                omega_sum += imus[j].gyro * dt;
            prev_t = imus[j].stamp_ns;
            j++;
        }

        double angle = omega_sum.norm();
        Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
        if (angle > 1e-8)
            dR = Eigen::AngleAxisd(angle, omega_sum.normalized()).toRotationMatrix();

        Eigen::Matrix4d dT = Eigen::Matrix4d::Identity();
        dT.block<3,3>(0,0) = dR;
        poses[i] = poses[i-1] * dT;
    }
    return poses;
}

// ═══════════════════════════════════════════════════════════════
// 3. ICP scan-to-scan
// ═══════════════════════════════════════════════════════════════

CloudT::Ptr downsample(CloudT::Ptr cloud, float leaf) {
    auto out = std::make_shared<CloudT>();
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*out);
    return out;
}

// Keep only wall/obstacle features for registration (not used in final output)
// Filters: z >= 0.2m (above ground) AND xy_range >= 0.3m (not robot body)
CloudT::Ptr wall_filter(CloudT::Ptr cloud) {
    auto out = std::make_shared<CloudT>();
    for (auto& p : *cloud) {
        float xy2 = p.x*p.x + p.y*p.y;
        if (p.z >= 0.2f && xy2 >= 0.09f)  // 0.3^2 = 0.09
            out->push_back(p);
    }
    return out;
}


CloudT::Ptr transform_cloud_by_pose(CloudT::Ptr cloud, const Eigen::Matrix4d& T) {
    auto out = std::make_shared<CloudT>();
    out->resize(cloud->size());
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);
    for (size_t i = 0; i < cloud->size(); i++) {
        Eigen::Vector3d p((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
        Eigen::Vector3d q = R * p + t;
        (*out)[i] = PointT(q.x(), q.y(), q.z());
    }
    return out;
}

std::vector<Eigen::Matrix4d> build_icp_poses(
        const std::vector<Scan>& scans,
        const std::vector<Eigen::Matrix4d>& imu_poses,
        float leaf = 0.15f, float max_dist = 1.5f) {
    int n = scans.size();
    std::vector<Eigen::Matrix4d> poses(n);
    poses[0] = Eigen::Matrix4d::Identity();

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setMaxCorrespondenceDistance(max_dist);
    gicp.setMaximumIterations(80);
    gicp.setTransformationEpsilon(1e-8);
    gicp.setMaximumOptimizerIterations(30);

    // Constant-velocity model
    Eigen::Matrix4d prev_delta = Eigen::Matrix4d::Identity();

    // Sliding-window local map
    const int W = 5;
    float map_leaf = leaf * 1.5f;

    for (int i = 1; i < n; i++) {
        // Source: wall-only features for better translation
        auto filtered = wall_filter(scans[i].cloud);
        auto src = downsample(filtered->size() >= 30 ? filtered : scans[i].cloud, leaf);

        // Target: FULL local map (ground+walls provide stability)
        CloudT::Ptr local_map(new CloudT);
        int start = std::max(0, i - W);
        for (int j = start; j < i; j++) {
            auto scan_g = transform_cloud_by_pose(
                downsample(scans[j].cloud, leaf), poses[j]);
            *local_map += *scan_g;
        }
        local_map = downsample(local_map, map_leaf);

        // IMU rotation + constant-velocity translation
        Eigen::Matrix4d T_rel_imu = imu_poses[i-1].inverse() * imu_poses[i];
        Eigen::Matrix4d T_init_rel = Eigen::Matrix4d::Identity();
        T_init_rel.block<3,3>(0,0) = T_rel_imu.block<3,3>(0,0);
        T_init_rel.block<3,1>(0,3) = prev_delta.block<3,1>(0,3);
        Eigen::Matrix4d init_g = poses[i-1] * T_init_rel;

        gicp.setInputSource(src);
        gicp.setInputTarget(local_map);

        CloudT result;
        gicp.align(result, init_g.cast<float>());

        if (gicp.hasConverged() && gicp.getFitnessScore() < max_dist) {
            poses[i] = gicp.getFinalTransformation().cast<double>();
        } else {
            poses[i] = init_g;
        }

        prev_delta = poses[i-1].inverse() * poses[i];

        if (i % 50 == 0 || i == n-1)
            std::cout << "    GICP " << i << "/" << n-1
                      << ": fitness=" << gicp.getFitnessScore()
                      << " map_pts=" << local_map->size()
                      << " converged=" << gicp.hasConverged() << std::endl;
    }
    return poses;
}



// ═══════════════════════════════════════════════════════════════
// 4. Loop closure detection
// ═══════════════════════════════════════════════════════════════

struct LoopClosure {
    int i, j;
    Eigen::Matrix4d T_j_from_i;  // relative transform
    double fitness;
};

std::vector<LoopClosure> detect_loop_closures(
        const std::vector<Scan>& scans,
        const std::vector<Eigen::Matrix4d>& poses,
        float leaf = 0.15f, int min_gap = 50,
        float proximity = 2.0f, float max_dist = 0.5f,
        double max_fitness = 0.15, int max_closures_per_scan = 3) {
    int n = poses.size();
    std::vector<LoopClosure> closures;

    // Build KD-tree of positions (XY)
    CloudT::Ptr pos_cloud(new CloudT);
    for (auto& p : poses)
        pos_cloud->push_back(PointT(p(0,3), p(1,3), 0));

    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(pos_cloud);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaxCorrespondenceDistance(max_dist);
    icp.setMaximumIterations(50);

    // Only check every 5th scan to reduce combinatorial explosion
    for (int i = min_gap; i < n; i += 5) {
        PointT query(poses[i](0,3), poses[i](1,3), 0);
        std::vector<int> idx;
        std::vector<float> dist;
        tree.radiusSearch(query, proximity, idx, dist);

        int closures_this_scan = 0;
        // Sort candidates by distance (closest first)
        std::vector<std::pair<float, int>> candidates;
        for (size_t k = 0; k < idx.size(); k++) {
            if (idx[k] < i - min_gap)
                candidates.push_back({dist[k], idx[k]});
        }
        std::sort(candidates.begin(), candidates.end());

        for (auto& [d, j] : candidates) {
            if (closures_this_scan >= max_closures_per_scan) break;

            auto src = downsample(scans[i].cloud, leaf);
            auto tgt = downsample(scans[j].cloud, leaf);
            auto src_g = transform_cloud_by_pose(src, poses[i]);
            auto tgt_g = transform_cloud_by_pose(tgt, poses[j]);

            icp.setInputSource(src_g);
            icp.setInputTarget(tgt_g);
            CloudT result;
            icp.align(result);

            if (icp.hasConverged() && icp.getFitnessScore() < max_fitness) {
                Eigen::Matrix4d T_icp = icp.getFinalTransformation().cast<double>();
                Eigen::Matrix4d T_rel = poses[j].inverse() * T_icp * poses[i];
                closures.push_back({i, j, T_rel, icp.getFitnessScore()});
                closures_this_scan++;
            }
        }
        if (i % 50 == 0)
            std::cout << "    Loop check " << i << "/" << n-1
                      << ": " << closures.size() << " closures" << std::endl;
    }
    std::cout << "  Total loop closures: " << closures.size() << std::endl;
    return closures;
}

// ═══════════════════════════════════════════════════════════════
// 5. Pose graph optimization (Ceres)
// ═══════════════════════════════════════════════════════════════

// Pose: translation[3] + angle-axis[3]
struct RelativePoseCost {
    Eigen::Vector3d t_meas;
    Eigen::Matrix3d R_meas;
    double weight;

    RelativePoseCost(const Eigen::Matrix4d& T_rel, double w)
        : t_meas(T_rel.block<3,1>(0,3)),
          R_meas(T_rel.block<3,3>(0,0)),
          weight(w) {}

    template <typename T>
    bool operator()(const T* const pose_i, const T* const pose_j,
                    T* residuals) const {
        // pose = [tx, ty, tz, ax, ay, az]
        // Compute R_i, t_i
        T R_i[9], R_j[9];
        ceres::AngleAxisToRotationMatrix(pose_i + 3, R_i);
        ceres::AngleAxisToRotationMatrix(pose_j + 3, R_j);

        // R_rel_pred = R_i^T * R_j
        // t_rel_pred = R_i^T * (t_j - t_i)
        for (int r = 0; r < 3; r++) {
            T dt = pose_j[r] - pose_i[r];
            T t_pred = T(0);
            for (int c = 0; c < 3; c++)
                t_pred += R_i[c * 3 + r] * (pose_j[c] - pose_i[c]);  // R_i^T
            residuals[r] = T(weight) * (t_pred - T(t_meas[r]));
        }

        // Rotation residual: log(R_meas^T * R_pred)
        T R_pred[9];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) {
                R_pred[r * 3 + c] = T(0);
                for (int k = 0; k < 3; k++)
                    R_pred[r * 3 + c] += R_i[k * 3 + r] * R_j[k * 3 + c];  // R_i^T * R_j
            }

        // R_err = R_meas^T * R_pred
        T R_err[9];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) {
                R_err[r * 3 + c] = T(0);
                for (int k = 0; k < 3; k++)
                    R_err[r * 3 + c] += T(R_meas(k, r)) * R_pred[k * 3 + c];
            }

        T aa[3];
        ceres::RotationMatrixToAngleAxis(R_err, aa);
        residuals[3] = T(weight) * aa[0];
        residuals[4] = T(weight) * aa[1];
        residuals[5] = T(weight) * aa[2];

        return true;
    }
};

struct GroundCost {
    double z0, weight;
    GroundCost(double z, double w) : z0(z), weight(w) {}

    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        residuals[0] = T(weight) * (pose[2] - T(z0));  // Z
        residuals[1] = T(weight) * pose[3];              // roll
        residuals[2] = T(weight) * pose[4];              // pitch
        return true;
    }
};

std::vector<Eigen::Matrix4d> optimize_pose_graph(
        const std::vector<Eigen::Matrix4d>& poses,
        const std::vector<LoopClosure>& loops) {
    int n = poses.size();

    // Parameterize: [tx, ty, tz, ax, ay, az] per pose
    std::vector<double> params(n * 6, 0.0);
    for (int i = 0; i < n; i++) {
        params[i*6+0] = poses[i](0,3);
        params[i*6+1] = poses[i](1,3);
        params[i*6+2] = poses[i](2,3);
        Eigen::AngleAxisd aa(poses[i].block<3,3>(0,0));
        Eigen::Vector3d rv = aa.angle() * aa.axis();
        params[i*6+3] = rv.x();
        params[i*6+4] = rv.y();
        params[i*6+5] = rv.z();
    }

    double z0 = params[2];

    ceres::Problem problem;

    // Odometry edges
    for (int i = 0; i < n-1; i++) {
        Eigen::Matrix4d T_rel = poses[i].inverse() * poses[i+1];
        auto cost = new ceres::AutoDiffCostFunction<RelativePoseCost, 6, 6, 6>(
            new RelativePoseCost(T_rel, 1.0));
        problem.AddResidualBlock(cost, nullptr, &params[i*6], &params[(i+1)*6]);
    }

    // Loop closure edges
    for (auto& lc : loops) {
        double w = 5.0;  // loop closures are strong constraints
        auto cost = new ceres::AutoDiffCostFunction<RelativePoseCost, 6, 6, 6>(
            new RelativePoseCost(lc.T_j_from_i, w));
        problem.AddResidualBlock(cost, nullptr, &params[lc.i*6], &params[lc.j*6]);
    }

    // Ground constraints
    for (int i = 0; i < n; i++) {
        auto cost = new ceres::AutoDiffCostFunction<GroundCost, 3, 6>(
            new GroundCost(z0, 2.0));
        problem.AddResidualBlock(cost, nullptr, &params[i*6]);
    }

    // Fix first pose
    problem.SetParameterBlockConstant(&params[0]);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 4;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    // Extract optimized poses
    std::vector<Eigen::Matrix4d> opt(n);
    for (int i = 0; i < n; i++) {
        opt[i] = Eigen::Matrix4d::Identity();
        opt[i](0,3) = params[i*6+0];
        opt[i](1,3) = params[i*6+1];
        opt[i](2,3) = params[i*6+2];
        Eigen::Vector3d rv(params[i*6+3], params[i*6+4], params[i*6+5]);
        double angle = rv.norm();
        if (angle > 1e-10)
            opt[i].block<3,3>(0,0) = Eigen::AngleAxisd(angle, rv/angle).toRotationMatrix();
    }
    return opt;
}

// ═══════════════════════════════════════════════════════════════
// 6. Output
// ═══════════════════════════════════════════════════════════════

void write_ply(const std::string& path, const std::vector<Scan>& scans,
               const std::vector<Eigen::Matrix4d>& poses, float ds_leaf = 0.05f) {
    CloudT::Ptr merged(new CloudT);
    for (size_t i = 0; i < scans.size(); i++) {
        auto cloud_g = transform_cloud_by_pose(scans[i].cloud, poses[i]);
        *merged += *cloud_g;
    }
    // Height filter
    CloudT::Ptr filtered(new CloudT);
    for (auto& p : *merged) {
        if (p.z > -0.5 && p.z < 2.5)
            filtered->push_back(p);
    }
    // Downsample
    auto out = downsample(filtered, ds_leaf);
    pcl::io::savePLYFileBinary(path, *out);
    std::cout << "  PLY saved: " << path << " (" << out->size() << " points)" << std::endl;
}

void write_trajectory(const std::string& path,
                      const std::vector<Eigen::Matrix4d>& before,
                      const std::vector<Eigen::Matrix4d>& after) {
    std::ofstream f(path);
    f << "# idx x_before y_before z_before x_after y_after z_after\n";
    for (size_t i = 0; i < before.size(); i++) {
        f << i
          << " " << before[i](0,3) << " " << before[i](1,3) << " " << before[i](2,3)
          << " " << after[i](0,3) << " " << after[i](1,3) << " " << after[i](2,3)
          << "\n";
    }
    f.close();
    std::cout << "  Trajectory saved: " << path << std::endl;
}

void export_for_balm2(const std::string& dir,
                       const std::vector<Scan>& scans,
                       const std::vector<Eigen::Matrix4d>& poses,
                       float ds_leaf = 0.1f) {
    // Create output directory
    std::string cmd = "mkdir -p " + dir;
    system(cmd.c_str());

    // Save each scan as PCD (body frame, downsampled)
    for (size_t i = 0; i < scans.size(); i++) {
        auto ds = downsample(scans[i].cloud, ds_leaf);
        // BALM2 uses PointXYZINormal
        pcl::PointCloud<pcl::PointXYZINormal> out;
        for (auto& p : *ds) {
            pcl::PointXYZINormal ap;
            ap.x = p.x; ap.y = p.y; ap.z = p.z;
            ap.intensity = 0; ap.curvature = 0;
            ap.normal_x = 0; ap.normal_y = 0; ap.normal_z = 0;
            out.push_back(ap);
        }
        std::string fname = dir + "/full" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(fname, out);
    }
    std::cout << "  " << scans.size() << " PCD files saved to " << dir << std::endl;

    // Save poses as CSV in BALM2 format:
    // 4 lines per pose, each line is 4 comma-separated values
    // The 4x4 matrix is stored ROW-MAJOR (transposed in BALM2's read_pose)
    // Last element of last row = timestamp
    std::ofstream f(dir + "/alidarPose.csv");
    for (size_t i = 0; i < poses.size(); i++) {
        Eigen::Matrix4d T = poses[i].transpose();  // BALM2 reads col-major and transposes
        T(3, 3) = (double)i;  // timestamp slot = index
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                f << T(r, c);
                if (c < 3) f << ",";
            }
            f << "\n";
        }
    }
    f.close();
    std::cout << "  Poses saved to " << dir << "/alidarPose.csv" << std::endl;
}

// ═══════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <bag_path> [--subsample N]" << std::endl;
        return 1;
    }

    std::string bag_path = argv[1];
    int subsample = 3;
    for (int i = 2; i < argc-1; i++) {
        if (std::string(argv[i]) == "--subsample")
            subsample = std::atoi(argv[i+1]);
    }

    std::string out_dir = ".";

    std::cout << std::string(60, '=') << std::endl;
    std::cout << "  LiDAR+IMU Scene Reconstruction (C++ / PCL / Ceres)" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    auto cal = load_calib();
    auto R_bc = body2cloud_rotation();

    // 1. Read bag
    std::cout << "\n1. Reading bag: " << bag_path << std::endl;
    auto t0 = std::chrono::steady_clock::now();
    std::vector<Scan> scans;
    std::vector<ImuReading> imus;
    read_bag(bag_path, subsample, cal, R_bc, scans, imus);
    auto dt = std::chrono::steady_clock::now() - t0;
    std::cout << "  Done in "
              << std::chrono::duration<double>(dt).count() << "s" << std::endl;

    if (scans.size() < 5) { std::cerr << "Not enough scans!" << std::endl; return 1; }

    // 2. IMU dead reckoning
    std::cout << "\n2. IMU dead reckoning (" << scans.size() << " scans)" << std::endl;
    t0 = std::chrono::steady_clock::now();
    auto imu_poses = imu_dead_reckon(scans, imus);
    dt = std::chrono::steady_clock::now() - t0;
    std::cout << "  Done in " << std::chrono::duration<double>(dt).count() << "s" << std::endl;

    // 3. ICP
    std::cout << "\n3. ICP scan-to-scan registration" << std::endl;
    t0 = std::chrono::steady_clock::now();
    auto icp_poses = build_icp_poses(scans, imu_poses, 0.15f, 1.0f);
    dt = std::chrono::steady_clock::now() - t0;
    std::cout << "  Done in " << std::chrono::duration<double>(dt).count() << "s" << std::endl;

    // 4. Loop closure
    std::cout << "\n4. Detecting loop closures" << std::endl;
    t0 = std::chrono::steady_clock::now();
    auto loops = detect_loop_closures(scans, icp_poses);
    dt = std::chrono::steady_clock::now() - t0;
    std::cout << "  Done in " << std::chrono::duration<double>(dt).count() << "s" << std::endl;

    // 5. PGO
    std::cout << "\n5. Pose graph optimization (" << loops.size() << " loops)" << std::endl;
    t0 = std::chrono::steady_clock::now();
    std::vector<Eigen::Matrix4d> opt_poses;
    if (!loops.empty()) {
        opt_poses = optimize_pose_graph(icp_poses, loops);
    } else {
        std::cout << "  No loop closures, using ICP poses" << std::endl;
        opt_poses = icp_poses;
    }
    dt = std::chrono::steady_clock::now() - t0;
    std::cout << "  Done in " << std::chrono::duration<double>(dt).count() << "s" << std::endl;

    // 6. Output
    std::cout << "\n6. Writing outputs" << std::endl;
    write_ply(out_dir + "/reconstructed_scene.ply", scans, opt_poses, 0.03f);
    write_trajectory(out_dir + "/trajectory.csv", icp_poses, opt_poses);

    // Export for BALM2 if requested
    bool export_balm = false;
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--export-balm") export_balm = true;
    if (export_balm) {
        std::cout << "\n7. Exporting for BALM2" << std::endl;
        export_for_balm2(out_dir + "/balm2_data", scans, opt_poses, 0.1f);
    }

    // Summary
    auto& tb = icp_poses;
    auto& ta = opt_poses;
    int last = ta.size() - 1;
    double gap_b = std::hypot(tb[last](0,3)-tb[0](0,3), tb[last](1,3)-tb[0](1,3));
    double gap_a = std::hypot(ta[last](0,3)-ta[0](0,3), ta[last](1,3)-ta[0](1,3));
    double ext_x = 0, ext_y = 0;
    double min_x=1e9, max_x=-1e9, min_y=1e9, max_y=-1e9;
    for (auto& p : ta) {
        min_x = std::min(min_x, p(0,3)); max_x = std::max(max_x, p(0,3));
        min_y = std::min(min_y, p(1,3)); max_y = std::max(max_y, p(1,3));
    }

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "  Arena extent:      " << max_x-min_x << "m × " << max_y-min_y << "m" << std::endl;
    std::cout << "  Loop gap (XY):     " << gap_b << "m → " << gap_a << "m" << std::endl;
    std::cout << "  Z drift:           " << std::abs(tb[last](2,3)-tb[0](2,3)) << "m → "
              << std::abs(ta[last](2,3)-ta[0](2,3)) << "m" << std::endl;
    std::cout << "  Loop closures:     " << loops.size() << std::endl;
    std::cout << "  Scans:             " << scans.size() << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    return 0;
}
