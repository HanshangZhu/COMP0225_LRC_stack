/**
 * run_balm2.cpp — Standalone BALM2 runner (no ROS dependency)
 *
 * Reads PCD files + pose CSV exported by reconstruct_scene,
 * runs BALM2 bundle adjustment (now with odometry regularization),
 * outputs refined PLY.
 *
 * Usage: ./run_balm2 <data_dir> [--voxel-size V] [--scale S]
 */

// Stub out ros::Time for bavoxel.hpp
#include <chrono>
namespace ros {
    struct TimeStub {
        double toSec() { 
            auto now = std::chrono::steady_clock::now();
            return std::chrono::duration<double>(now.time_since_epoch()).count();
        }
    };
    struct Time { static TimeStub now() { return TimeStub{}; } };
}

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

// BALM2 core headers (with ros:: stubbed above)
#include "tools.hpp"
#include "bavoxel.hpp"

using namespace std;

// ═══════════════════════════════════════════════════════════════
// Read data in BALM2 format
// ═══════════════════════════════════════════════════════════════

int read_pose(vector<double>& tims, PLM(3)& rots, PLV(3)& poss, const string& prename) {
    string readname = prename + "alidarPose.csv";
    ifstream inFile(readname);
    if (!inFile.is_open()) {
        printf("Cannot open %s\n", readname.c_str());
        return 0;
    }
    int pose_size = 0;
    string lineStr, str;
    Eigen::Matrix4d aff;
    vector<double> nums;
    int ord = 0;
    while (getline(inFile, lineStr)) {
        ord++;
        stringstream ss(lineStr);
        while (getline(ss, str, ','))
            nums.push_back(stod(str));
        if (ord == 4) {
            for (int j = 0; j < 16; j++)
                aff(j) = nums[j];

            // aff is loaded column-major from row-major CSV of pose^T
            // This means: aff(row, col) = pose_T(col, row) = pose(row, col)
            // So aff = pose (the original SE(3) matrix)!
            rots.push_back(aff.block<3, 3>(0, 0));  // rotation from pose directly
            poss.push_back(aff.block<3, 1>(0, 3));   // position from column 3 of pose
            
            Eigen::Matrix4d affT = aff.transpose();
            tims.push_back(affT(3, 3));  // timestamp stored at T(3,3) = pose^T(3,3)
            nums.clear();
            ord = 0;
            pose_size++;
        }
    }
    return pose_size;
}

void read_data(vector<IMUST>& x_buf,
               vector<pcl::PointCloud<PointType>::Ptr>& pl_fulls,
               const string& dir) {
    PLV(3) poss; PLM(3) rots;
    vector<double> tims;
    int pose_size = read_pose(tims, rots, poss, dir);
    printf("  Read %d poses\n", pose_size);
    for (int m = 0; m < pose_size; m++) {
        string filename = dir + "full" + to_string(m) + ".pcd";
        pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
        if (pcl::io::loadPCDFile(filename, *pl_ptr) < 0)
            pl_ptr->clear();
        pl_fulls.push_back(pl_ptr);
        IMUST curr;
        curr.R = rots[m]; curr.p = poss[m]; curr.t = tims[m];
        x_buf.push_back(curr);
    }
}

// ═══════════════════════════════════════════════════════════════
// Output
// ═══════════════════════════════════════════════════════════════

void save_result(const vector<IMUST>& x_buf,
                 const vector<pcl::PointCloud<PointType>::Ptr>& pl_fulls,
                 const string& ply_path, float ds = 0.03f) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < x_buf.size(); i++) {
        pcl::PointCloud<PointType> pl_tem = *pl_fulls[i];
        pl_transform(pl_tem, x_buf[i]);
        for (auto& p : pl_tem)
            if (p.z > -0.5 && p.z < 2.5)
                merged->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(merged);
    vg.setLeafSize(ds, ds, ds);
    vg.filter(*out);
    pcl::io::savePLYFileBinary(ply_path, *out);
    printf("  PLY saved: %s (%zu points)\n", ply_path.c_str(), out->size());
}

void save_trajectory(const vector<IMUST>& x_buf, const string& path) {
    ofstream f(path);
    f << "# idx x y z\n";
    for (size_t i = 0; i < x_buf.size(); i++)
        f << i << " " << x_buf[i].p.x() << " " << x_buf[i].p.y() << " " << x_buf[i].p.z() << "\n";
    f.close();
    printf("  Trajectory saved: %s\n", path.c_str());
}

// ═══════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s <data_dir> [--voxel-size V] [--scale S]\n", argv[0]);
        return 1;
    }

    string data_dir = string(argv[1]);
    if (data_dir.back() != '/') data_dir += "/";

    voxel_size = 1.0;
    double scale = 1.0;
    for (int i = 2; i < argc - 1; i++) {
        if (string(argv[i]) == "--voxel-size")
            voxel_size = atof(argv[i+1]);
        if (string(argv[i]) == "--scale")
            scale = atof(argv[i+1]);
    }

    printf("============================================================\n");
    printf("  BALM2 Bundle Adjustment (with odometry regularization)\n");
    printf("  Voxel size: %.1f m, Scale: %.1f\n", voxel_size, scale);
    printf("============================================================\n");

    // 1. Read data
    printf("\n1. Reading data from %s\n", data_dir.c_str());
    vector<IMUST> x_buf;
    vector<pcl::PointCloud<PointType>::Ptr> pl_fulls;
    read_data(x_buf, pl_fulls, data_dir);

    if (x_buf.size() < 3) {
        printf("Not enough scans!\n");
        return 1;
    }

    win_size = x_buf.size();
    printf("  %d scans loaded\n", win_size);

    // Normalize to origin
    IMUST es0 = x_buf[0];
    for (uint i = 0; i < x_buf.size(); i++) {
        x_buf[i].p = es0.R.transpose() * (x_buf[i].p - es0.p);
        x_buf[i].R = es0.R.transpose() * x_buf[i].R;
    }

    // Apply scale correction to translations (keep rotations)
    if (fabs(scale - 1.0) > 0.01) {
        printf("  Applying scale factor %.1fx to translations\n", scale);
        for (uint i = 0; i < x_buf.size(); i++)
            x_buf[i].p *= scale;
    }

    // Save before state
    save_trajectory(x_buf, "trajectory_before_ba.csv");

    // 2. Build voxel map + find planes
    printf("\n2. Building voxel map (voxel_size=%.1f)\n", voxel_size);

    // Relaxed plane detection
    eigen_value_array[0] = 1.0 / 4;
    eigen_value_array[1] = 1.0 / 4;
    eigen_value_array[2] = 1.0 / 4;
    layer_limit = 2;
    min_ps = 5;

    unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;

    for (int i = 0; i < win_size; i++)
        cut_voxel(surf_map, *pl_fulls[i], x_buf[i], i);

    VOX_HESS voxhess;
    for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++) {
        iter->second->recut(win_size);
        iter->second->tras_opt(voxhess, win_size);
    }

    printf("  %zu plane voxels found\n", voxhess.plvec_voxels.size());

    if (voxhess.plvec_voxels.size() < 3 * (size_t)win_size) {
        printf("  WARNING: Few planes (%zu < %d). Retrying with larger voxels...\n",
               voxhess.plvec_voxels.size(), 3 * win_size);

        for (auto iter = surf_map.begin(); iter != surf_map.end();)
            { delete iter->second; surf_map.erase(iter++); }

        voxel_size *= 2;
        min_ps = 3;

        for (int i = 0; i < win_size; i++)
            cut_voxel(surf_map, *pl_fulls[i], x_buf[i], i);

        voxhess.plvec_voxels.clear();
        voxhess.sig_vecs.clear();
        voxhess.coeffs.clear();
        voxhess.plptrs.clear();

        for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++) {
            iter->second->recut(win_size);
            iter->second->tras_opt(voxhess, win_size);
        }

        printf("  Retry: %zu plane voxels (voxel_size=%.1f)\n",
               voxhess.plvec_voxels.size(), voxel_size);
    }

    // Check per-scan plane count before calling damping_iter (it has exit(0))
    {
        vector<int> planes(win_size, 0);
        for (size_t a = 0; a < voxhess.plvec_voxels.size(); a++)
            for (int j = 0; j < win_size; j++)
                if ((*voxhess.plvec_voxels[a])[j].N != 0)
                    planes[j]++;
        sort(planes.begin(), planes.end());
        printf("  Min planes per scan: %d (need ≥20)\n", planes[0]);
        if (planes[0] < 20) {
            printf("  ERROR: Not enough planes per scan for BA. Skipping.\n");
            // Still save the un-optimized result
            for (uint i = 0; i < x_buf.size(); i++) {
                x_buf[i].p = es0.R * x_buf[i].p + es0.p;
                x_buf[i].R = es0.R * x_buf[i].R;
            }
            save_result(x_buf, pl_fulls, "reconstructed_scene_ba.ply", 0.03f);
            save_trajectory(x_buf, "trajectory_after_ba.csv");
            return 1;
        }
    }

    // 3. Run BALM2 optimization (now with odometry regularization!)
    printf("\n3. Running BALM2 optimization\n");
    auto t0 = chrono::steady_clock::now();

    BALM2 opt_lsv;
    opt_lsv.damping_iter(x_buf, voxhess);

    auto dt = chrono::steady_clock::now() - t0;
    printf("  Done in %.2fs\n", chrono::duration<double>(dt).count());

    // Cleanup
    for (auto iter = surf_map.begin(); iter != surf_map.end();)
        { delete iter->second; surf_map.erase(iter++); }

    // 4. Save results
    printf("\n4. Saving results\n");

    // Un-normalize
    for (uint i = 0; i < x_buf.size(); i++) {
        x_buf[i].p = es0.R * x_buf[i].p + es0.p;
        x_buf[i].R = es0.R * x_buf[i].R;
    }

    save_result(x_buf, pl_fulls, "reconstructed_scene_ba.ply", 0.03f);
    save_trajectory(x_buf, "trajectory_after_ba.csv");

    // Summary
    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (auto& s : x_buf) {
        min_x = min(min_x, s.p.x()); max_x = max(max_x, s.p.x());
        min_y = min(min_y, s.p.y()); max_y = max(max_y, s.p.y());
    }
    double gap = (x_buf.back().p - x_buf.front().p).head<2>().norm();

    printf("\n============================================================\n");
    printf("  Arena extent:   %.1fm × %.1fm\n", max_x-min_x, max_y-min_y);
    printf("  Loop gap (XY):  %.2fm\n", gap);
    printf("  Z drift:        %.3fm\n", abs(x_buf.back().p.z() - x_buf.front().p.z()));
    printf("============================================================\n");

    return 0;
}
