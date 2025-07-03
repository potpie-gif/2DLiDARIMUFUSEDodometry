#ifndef LIDAR_ODOMETRY_H
#define LIDAR_ODOMETRY_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/registration/gicp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <deque>
#include <map>
#include "lidar_odometry/utils.hpp"

class LidarOdometry
{
    public:
        LidarOdometry(double max_correspondence_distance = 1.0, double transformation_epsilon = 0.001, double maximum_iterations = 1000);
        StatePtr get_state();
        void process_scan_data(const ScanDataPtr scan_data);
        void process_imu_data(const ImuDataPtr imu_data);
        void update_pose_with_fusion(const PoseEstimate& fused_estimate);
    private:

        ScanDataPtr last_scan_ptr;
        ImuDataPtr latest_imu_ptr;
        StatePtr state_ptr;
        Eigen::Matrix4d POSE_G_L; // pose in SE(3) (ground -> LiDAR)

        // Motion prediction and fusion
        Eigen::Vector3d predicted_translation;
        double predicted_yaw;
        double last_imu_timestamp;
        bool has_previous_imu;
        
        // Pose estimates for fusion
        PoseEstimate lidar_pose_estimate;
        PoseEstimate imu_pose_estimate;
        
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr gicp;
        
        bool has_imu_data;
        bool has_lidar_data;
        
        // Drift correction parameters
        double translation_threshold;
        double rotation_threshold;
        double velocity_threshold;
        
        // Previous velocities for stationary detection
        Eigen::Vector3d prev_linear_velocity;
        Eigen::Vector3d prev_angular_velocity;
        
        // Stationary state tracking
        bool is_robot_stationary;
        int stationary_count;
        
        // Tilt and ground detection state
        bool is_robot_tilted;
        int consecutive_ground_scans;
        double max_tilt_angle_rad;
        double ground_detection_threshold;
        
        // Loop closure and keyframe management
        struct Keyframe {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            Eigen::Isometry3d pose;
            std::vector<float> scan_context;
            double timestamp;
            int id;
        };
        
        std::vector<Keyframe> keyframes;
        double keyframe_distance_threshold;
        double keyframe_angle_threshold;
        int last_keyframe_id;
        
        // Scan Context parameters
        static constexpr int SC_RING_NUM = 20;
        static constexpr int SC_SECTOR_NUM = 60;
        static constexpr double SC_MAX_RADIUS = 20.0;
        
        // Loop closure detection
        struct LoopCandidate {
            int keyframe_id;
            double similarity_score;
            Eigen::Matrix4d relative_transform;
        };
        
        std::deque<LoopCandidate> loop_candidates;
        double loop_closure_threshold;
        int min_keyframes_between_loops;
        
        // Pose graph optimization
        struct PoseNode {
            Eigen::Isometry3d pose;
            bool is_fixed;
        };
        
        struct PoseEdge {
            int from_id;
            int to_id;
            Eigen::Matrix4d relative_transform;
            Eigen::Matrix<double, 6, 6> information_matrix;
            bool is_loop_closure;
        };
        
        std::map<int, PoseNode> pose_graph_nodes;
        std::vector<PoseEdge> pose_graph_edges;
        bool pose_graph_needs_optimization;

        Eigen::Matrix4d get_transform_matrix(ScanDataPtr source, ScanDataPtr target, const Eigen::Matrix4d& initial_guess = Eigen::Matrix4d::Identity());
        Eigen::Vector3d extract_translation_only(const Eigen::Matrix4d& transform);
        bool detect_stationary_robot(const Eigen::Vector3d& linear_vel, const Eigen::Vector3d& angular_vel);
        void remove_ground_points(pcl::PointCloud<pcl::PointXYZ>& cloud);
        Eigen::Vector3d estimate_ground_plane_normal(const pcl::PointCloud<pcl::PointXYZ>& cloud);
        void compensate_for_terrain_inclination(pcl::PointCloud<pcl::PointXYZ>& cloud);
        void filter_points_by_height(pcl::PointCloud<pcl::PointXYZ>& cloud);
        
        // IMU prediction and fusion methods
        void predict_motion_from_imu(const ImuDataPtr& current_imu, const ImuDataPtr& previous_imu);
        Eigen::Matrix4d create_initial_guess(double dx, double dy, double dyaw);
        PoseEstimate fuse_pose_estimates(const PoseEstimate& lidar_est, const PoseEstimate& imu_est);
        double extract_yaw_from_transform(const Eigen::Matrix4d& transform);
        void integrate_imu_motion(const ImuDataPtr& imu_data, double dt);
        
        // Enhanced tilt and ground handling methods
        bool detect_excessive_tilt(const ImuDataPtr& imu_data);
        bool is_seeing_mostly_ground(const pcl::PointCloud<pcl::PointXYZ>& cloud);
        void enhanced_ground_plane_removal(pcl::PointCloud<pcl::PointXYZ>& cloud);
        bool has_sufficient_features(const pcl::PointCloud<pcl::PointXYZ>& cloud);
        Eigen::Matrix3d get_tilt_compensation_matrix(const ImuDataPtr& imu_data);
        void apply_tilt_compensation_to_cloud(pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Matrix3d& compensation);
        void update_pose_with_imu_only();
        bool validate_pose_estimate(const PoseEstimate& estimate, double dt);
        
        // Loop closure and keyframe methods
        std::vector<float> compute_scan_context(const pcl::PointCloud<pcl::PointXYZ>& cloud);
        double compute_scan_context_distance(const std::vector<float>& sc1, const std::vector<float>& sc2);
        bool should_create_keyframe(const Eigen::Isometry3d& current_pose);
        void create_keyframe(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& pose, double timestamp);
        std::vector<int> detect_loop_closure_candidates(const std::vector<float>& current_sc);
        bool verify_loop_closure(int candidate_id, const pcl::PointCloud<pcl::PointXYZ>& current_cloud, Eigen::Matrix4d& relative_transform);
        void perform_loop_closure(int matched_keyframe_id, const Eigen::Matrix4d& relative_transform);
        
        // Pose graph optimization methods
        void add_pose_node(int id, const Eigen::Isometry3d& pose, bool is_fixed = false);
        void add_pose_edge(int from_id, int to_id, const Eigen::Matrix4d& relative_transform, bool is_loop_closure = false);
        void optimize_pose_graph();
        Eigen::Matrix<double, 6, 6> compute_information_matrix(const Eigen::Matrix4d& transform, bool is_loop_closure);
        void update_poses_from_graph();
};

#endif
