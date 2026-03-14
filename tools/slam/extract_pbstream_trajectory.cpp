// extract_pbstream_trajectory.cpp
// Extracts trajectory node poses from a Cartographer .pbstream file
// Handles the fact that node poses are local — applies submap transform
#include <cartographer/io/proto_stream.h>
#include <cartographer/io/proto_stream_deserializer.h>
#include <cartographer/mapping/proto/serialization.pb.h>
#include <cartographer/mapping/proto/pose_graph.pb.h>
#include <cartographer/transform/transform.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <Eigen/Core>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <pbstream> [output.csv]" << std::endl;
        return 1;
    }
    
    std::string pbstream_path = argv[1];
    std::string output_path = argc > 2 ? argv[2] : "trajectory_from_pbstream.csv";
    
    // Read pbstream
    cartographer::io::ProtoStreamReader reader(pbstream_path);
    cartographer::io::ProtoStreamDeserializer deserializer(&reader);
    
    // Get pose graph
    const auto& pose_graph = deserializer.pose_graph();
    
    std::cout << "Trajectory count: " << pose_graph.trajectory_size() << std::endl;
    
    // 1. Extract trajectory nodes (local poses)
    struct Node { int traj_id, node_id; Eigen::Vector3d pos; Eigen::Quaterniond rot; };
    std::vector<Node> nodes;
    for (int t = 0; t < pose_graph.trajectory_size(); t++) {
        const auto& traj = pose_graph.trajectory(t);
        std::cout << "  Trajectory " << t << ": " << traj.node_size() << " nodes, "
                  << traj.submap_size() << " submaps" << std::endl;
        for (int n = 0; n < traj.node_size(); n++) {
            const auto& node = traj.node(n);
            auto p = cartographer::transform::ToRigid3(node.pose());
            nodes.push_back({t, node.node_index(), p.translation(),
                            Eigen::Quaterniond(p.rotation())});
        }
    }
    
    // 2. Extract submap poses
    struct Submap { int traj_id, submap_idx; Eigen::Vector3d pos; Eigen::Quaterniond rot; };
    std::vector<Submap> submaps;
    for (int t = 0; t < pose_graph.trajectory_size(); t++) {
        const auto& traj = pose_graph.trajectory(t);
        for (int s = 0; s < traj.submap_size(); s++) {
            const auto& sm = traj.submap(s);
            auto p = cartographer::transform::ToRigid3(sm.pose());
            submaps.push_back({t, sm.submap_index(), p.translation(),
                              Eigen::Quaterniond(p.rotation())});
        }
    }
    
    // 3. Extract constraints (which node belongs to which submap)
    std::cout << "Constraints: " << pose_graph.constraint_size() << std::endl;
    
    // Build a map: node_id -> {submap_id, relative_pose}
    // For trajectory reconstruction, find each node's containing submap
    struct Constraint {
        int submap_traj, submap_idx, node_traj, node_idx;
        Eigen::Vector3d t_rel;
        Eigen::Quaterniond r_rel;
        bool is_intra;
    };
    std::vector<Constraint> constraints;
    std::map<int, int> node_to_submap; // node_idx -> submap_idx (intra-submap)
    std::map<int, Eigen::Matrix4d> node_to_relative; // relative pose in submap
    
    for (int c = 0; c < pose_graph.constraint_size(); c++) {
        const auto& con = pose_graph.constraint(c);
        auto p = cartographer::transform::ToRigid3(con.relative_pose());
        bool intra = (con.tag() == cartographer::mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP);
        
        if (intra) {
            int ni = con.node_id().node_index();
            int si = con.submap_id().submap_index();
            node_to_submap[ni] = si;
            
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3,3>(0,0) = p.rotation().toRotationMatrix();
            T.block<3,1>(0,3) = p.translation();
            node_to_relative[ni] = T;
        }
    }
    
    // 4. Compute global poses: global_node = submap_global * relative_in_submap
    // Build submap pose lookup
    std::map<int, Eigen::Matrix4d> submap_global;
    for (auto& sm : submaps) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = sm.rot.toRotationMatrix();
        T.block<3,1>(0,3) = sm.pos;
        submap_global[sm.submap_idx] = T;
    }
    
    std::ofstream out(output_path);
    out << "node_id,x_local,y_local,z_local,x_global,y_global,z_global,gqw,gqx,gqy,gqz" << std::endl;
    
    int computed = 0;
    for (auto& node : nodes) {
        double gx = node.pos.x(), gy = node.pos.y(), gz = node.pos.z();
        Eigen::Quaterniond gq = node.rot;
        
        // Try to compute global pose via submap constraint
        auto it = node_to_submap.find(node.node_id);
        if (it != node_to_submap.end()) {
            auto sit = submap_global.find(it->second);
            if (sit != submap_global.end()) {
                Eigen::Matrix4d global = sit->second * node_to_relative[node.node_id];
                gx = global(0,3); gy = global(1,3); gz = global(2,3);
                Eigen::Matrix3d R = global.block<3,3>(0,0);
                gq = Eigen::Quaterniond(R);
                computed++;
            }
        }
        
        out << node.node_id << "," 
            << node.pos.x() << "," << node.pos.y() << "," << node.pos.z() << ","
            << gx << "," << gy << "," << gz << ","
            << gq.w() << "," << gq.x() << "," << gq.y() << "," << gq.z() << std::endl;
    }
    
    std::cout << "Wrote " << nodes.size() << " nodes (" << computed << " with global poses) to " 
              << output_path << std::endl;
    return 0;
}
