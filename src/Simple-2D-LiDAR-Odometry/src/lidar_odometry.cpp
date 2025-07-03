
#include "lidar_odometry/lidar_odometry.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>
#include <algorithm>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define the missing typedef for Eigen::Vector6d
using Vector6d = Eigen::Matrix<double, 6, 1>;

LidarOdometry::LidarOdometry(double max_correspondence_distance, double transformation_epsilon, double maximum_iterations)
{
    gicp = std::make_shared<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();

    // Improved GICP parameters for better loop closure detection
    gicp->setMaxCorrespondenceDistance(std::max(1.5, max_correspondence_distance)); // Better long-range matching
    gicp->setTransformationEpsilon(transformation_epsilon * 0.1); // Sensitive convergence
    gicp->setMaximumIterations(maximum_iterations);
    gicp->setRANSACIterations(100);             // More robust outlier rejection
    gicp->setRANSACOutlierRejectionThreshold(0.15); // More permissive for diverse viewpoints
    gicp->setEuclideanFitnessEpsilon(0.001);    // Balanced convergence threshold
    gicp->setUseReciprocalCorrespondences(true); // Better correspondence matching

    // Initialize motion prediction variables
    predicted_translation = Eigen::Vector3d::Zero();
    predicted_yaw = 0.0;
    last_imu_timestamp = 0.0;
    has_previous_imu = false;
    
    has_imu_data = false;
    has_lidar_data = false;
    
    // Initialize pose estimates
    lidar_pose_estimate = {Eigen::Vector3d::Zero(), 0.0, 0.0, 0.0};
    imu_pose_estimate = {Eigen::Vector3d::Zero(), 0.0, 0.0, 0.0};
    
    // Initialize drift correction parameters - optimized for 4km/h robot
    translation_threshold = 0.002;  // 2mm threshold for high precision
    rotation_threshold = 0.008;     // ~0.5 degree threshold (more sensitive)
    velocity_threshold = 0.3;       // 30cm/s threshold for 4km/h robot
    
    // Initialize velocity tracking
    prev_linear_velocity = Eigen::Vector3d::Zero();
    prev_angular_velocity = Eigen::Vector3d::Zero();
    
    // Initialize stationary detection
    is_robot_stationary = false;
    stationary_count = 0;
    
    // Initialize tilt and ground detection
    is_robot_tilted = false;
    consecutive_ground_scans = 0;
    max_tilt_angle_rad = 0.35; // ~20 degrees maximum tilt
    ground_detection_threshold = 0.7; // 70% of points must be ground to trigger
    
    // Initialize loop closure parameters
    keyframe_distance_threshold = 0.5; // 50cm between keyframes
    keyframe_angle_threshold = 0.2; // ~11 degrees rotation threshold
    last_keyframe_id = -1;
    loop_closure_threshold = 0.15; // Scan context similarity threshold
    min_keyframes_between_loops = 30; // Minimum keyframes before considering loop closure
    pose_graph_needs_optimization = false;

    POSE_G_L = Eigen::Matrix4d::Identity();

    state_ptr = std::make_shared<State>();
    state_ptr->pose = Eigen::Isometry3d::Identity();
    state_ptr->velocity = Eigen::Matrix<double, 6, 1>::Zero();
    state_ptr->timestamp = 0.0;
}

StatePtr LidarOdometry::get_state()
{
    return state_ptr;
}

void LidarOdometry::process_scan_data(ScanDataPtr scan_ptr)
{
    if (last_scan_ptr) {
        double dt = scan_ptr->timestamp - last_scan_ptr->timestamp;
        
        // Check for excessive tilt that would make LiDAR unreliable
        bool robot_tilted = has_imu_data && latest_imu_ptr && detect_excessive_tilt(latest_imu_ptr);
        bool mostly_ground = is_seeing_mostly_ground(scan_ptr->point_cloud);
        bool sufficient_features = has_sufficient_features(scan_ptr->point_cloud);
        
        // Skip odometry update if conditions are poor for reliable estimation
        if (robot_tilted || mostly_ground || !sufficient_features) {
            consecutive_ground_scans++;
            
            // Use pure IMU prediction when LiDAR is unreliable
            if (has_imu_data && has_previous_imu && consecutive_ground_scans < 10) {
                // Apply IMU-only motion prediction
                update_pose_with_imu_only();
            }
            
            last_scan_ptr = scan_ptr;
            return;
        }
        
        // Reset ground scan counter if we have good data
        consecutive_ground_scans = 0;
        
        // Create initial guess from IMU prediction if available
        Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
        if (has_imu_data && has_previous_imu) {
            initial_guess = create_initial_guess(predicted_translation.x(), predicted_translation.y(), predicted_yaw);
        }
        
        Eigen::Matrix4d transform_matrix = get_transform_matrix(last_scan_ptr, scan_ptr, initial_guess);
        
        
        if (gicp->hasConverged() && gicp->getFitnessScore() < 0.1) { // More permissive fitness threshold
            // Extract 2D pose from LiDAR scan matching
            Eigen::Vector3d lidar_translation = transform_matrix.block<3, 1>(0, 3);
            double lidar_yaw = extract_yaw_from_transform(transform_matrix);
            
            // Validate transformation magnitude with reasonable limits
            if (lidar_translation.norm() < 0.5 && std::abs(lidar_yaw) < 0.5) { // Allow larger movements for better tracking
                // Create LiDAR pose estimate
                lidar_pose_estimate.translation = lidar_translation;
                lidar_pose_estimate.yaw = lidar_yaw;
                lidar_pose_estimate.timestamp = scan_ptr->timestamp;
                double fitness_score = gicp->getFitnessScore();
                double normalized_fitness = std::min(0.95, fitness_score / 0.05);
                lidar_pose_estimate.confidence = 1.0 - normalized_fitness;
                
                // Fuse LiDAR and IMU estimates
                PoseEstimate fused_estimate = fuse_pose_estimates(lidar_pose_estimate, imu_pose_estimate);
                
                // Validate pose estimate before applying
                if (validate_pose_estimate(fused_estimate, dt)) {
                    // Update robot state with fused estimate
                    update_pose_with_fusion(fused_estimate);
                    
                    // Loop closure detection and keyframe management
                    if (should_create_keyframe(state_ptr->pose)) {
                        create_keyframe(scan_ptr->point_cloud, state_ptr->pose, scan_ptr->timestamp);
                        
                        // Detect loop closures if we have enough keyframes
                        if (keyframes.size() > static_cast<size_t>(min_keyframes_between_loops)) {
                            std::vector<float> current_sc = compute_scan_context(scan_ptr->point_cloud);
                            std::vector<int> candidates = detect_loop_closure_candidates(current_sc);
                            
                            for (int candidate_id : candidates) {
                                Eigen::Matrix4d relative_transform;
                                if (verify_loop_closure(candidate_id, scan_ptr->point_cloud, relative_transform)) {
                                    perform_loop_closure(candidate_id, relative_transform);
                                    break; // Only process one loop closure per scan
                                }
                            }
                        }
                    }
                    
                    // Optimize pose graph if needed
                    if (pose_graph_needs_optimization) {
                        optimize_pose_graph();
                        update_poses_from_graph();
                        pose_graph_needs_optimization = false;
                    }
                } else {
                    // Use IMU-only prediction if pose estimate is invalid
                    if (has_imu_data && has_previous_imu) {
                        update_pose_with_imu_only();
                    }
                }
                
                // Calculate velocity for next prediction
                Eigen::Vector3d new_linear_velocity = fused_estimate.translation / dt;
                double new_angular_velocity = fused_estimate.yaw / dt;
                
                // Smooth velocities with minimal filtering for maximum responsiveness
                double alpha = 0.4; // Increased for even faster response to rotation changes
                state_ptr->velocity.block<3, 1>(0, 0) = 
                    alpha * new_linear_velocity + (1 - alpha) * prev_linear_velocity;
                state_ptr->velocity[5] = alpha * new_angular_velocity + (1 - alpha) * prev_angular_velocity[2];
                
                prev_linear_velocity = state_ptr->velocity.block<3, 1>(0, 0);
                prev_angular_velocity[2] = state_ptr->velocity[5];
                state_ptr->timestamp = scan_ptr->timestamp;
            }
        }
        
        has_lidar_data = true;
    }

    last_scan_ptr = scan_ptr;
}

void LidarOdometry::process_imu_data(const ImuDataPtr imu_data)
{
    if (has_previous_imu && latest_imu_ptr) {
        // Predict motion using IMU data
        predict_motion_from_imu(imu_data, latest_imu_ptr);
        
        // Update IMU pose estimate with integrated motion
        double dt = imu_data->timestamp - latest_imu_ptr->timestamp;
        integrate_imu_motion(imu_data, dt);
    }
    
    // Store current IMU data for next iteration
    latest_imu_ptr = imu_data;
    last_imu_timestamp = imu_data->timestamp;
    has_imu_data = true;
    has_previous_imu = true;
}

void LidarOdometry::update_pose_with_fusion(const PoseEstimate& fused_estimate)
{
    // Apply fused pose estimate with drift correction
    Eigen::Vector3d translation_increment = fused_estimate.translation;
    
    // Minimal filtering for better responsiveness on slow robots
    if (translation_increment.norm() < 0.0005) {
        translation_increment = Eigen::Vector3d::Zero(); // Only filter sub-0.5mm noise
    } else if (translation_increment.norm() < 0.002) {
        translation_increment *= 0.9; // Minimal reduction for small movements
    }
    
    
    // Constrain Z-axis movement for ground robot (prevent vertical drift)
    translation_increment.z() = 0.0;
    
    POSE_G_L.block<3, 1>(0, 3) += translation_increment;
    
    // Force Z position to stay at ground level (prevent accumulation)
    POSE_G_L(2, 3) = 0.0;
    
    // Apply yaw rotation
    Eigen::AngleAxisd yaw_rotation(fused_estimate.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotation_matrix = yaw_rotation.toRotationMatrix();
    POSE_G_L.block<3, 3>(0, 0) = POSE_G_L.block<3, 3>(0, 0) * rotation_matrix;
    
    // Update state from pose matrix
    state_ptr->pose.translation() = POSE_G_L.block<3, 1>(0, 3);
    state_ptr->pose.linear() = POSE_G_L.block<3, 3>(0, 0);
    state_ptr->timestamp = fused_estimate.timestamp;
    
    // Reset prediction for next cycle
    predicted_translation = Eigen::Vector3d::Zero();
    predicted_yaw = 0.0;
}

Eigen::Matrix4d LidarOdometry::get_transform_matrix(ScanDataPtr source, ScanDataPtr target, const Eigen::Matrix4d& initial_guess)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (source->point_cloud.points.empty() || target->point_cloud.points.empty()) {
        return Eigen::Matrix4d::Identity();
    }

    if (source->point_cloud.points.size() < 10 || target->point_cloud.points.size() < 10) {
        return Eigen::Matrix4d::Identity();
    }

    // Create copies for inclination compensation and ground filtering
    pcl::PointCloud<pcl::PointXYZ> source_corrected = source->point_cloud;
    pcl::PointCloud<pcl::PointXYZ> target_corrected = target->point_cloud;
    
    // Apply enhanced tilt compensation if IMU data available
    if (has_imu_data && latest_imu_ptr) {
        Eigen::Matrix3d tilt_compensation = get_tilt_compensation_matrix(latest_imu_ptr);
        apply_tilt_compensation_to_cloud(source_corrected, tilt_compensation);
        apply_tilt_compensation_to_cloud(target_corrected, tilt_compensation);
    }
    
    compensate_for_terrain_inclination(source_corrected);
    compensate_for_terrain_inclination(target_corrected);
    
    // Use enhanced ground plane removal
    enhanced_ground_plane_removal(source_corrected);
    enhanced_ground_plane_removal(target_corrected);
    
    // Additional height-based filtering for extra robustness
    filter_points_by_height(source_corrected);
    filter_points_by_height(target_corrected);

    
    // Validate filtered point clouds before GICP
    if (source_corrected.points.empty() || target_corrected.points.empty()) {
        return Eigen::Matrix4d::Identity();
    }
    
    if (source_corrected.points.size() < 10 || target_corrected.points.size() < 10) {
        return Eigen::Matrix4d::Identity();
    }

    // Create shared pointers for GICP input
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(source_corrected);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(target_corrected);
    
    // Ensure point clouds have proper structure
    source_ptr->width = source_ptr->points.size();
    source_ptr->height = 1;
    source_ptr->is_dense = false;
    
    target_ptr->width = target_ptr->points.size();
    target_ptr->height = 1;
    target_ptr->is_dense = false;

    gicp->setInputSource(source_ptr);
    gicp->setInputTarget(target_ptr);
    
    // Use IMU prediction as initial guess for GICP
    gicp->align(*align, initial_guess.cast<float>());

    Eigen::Matrix4f src2tgt = gicp->getFinalTransformation();
    return src2tgt.cast<double>();    
}

Eigen::Vector3d LidarOdometry::extract_translation_only(const Eigen::Matrix4d& transform)
{
    // Extract only the translation component, ignore rotation completely
    // This ensures LiDAR ONLY contributes to translation (X,Y,Z movement)
    // IMU will handle all rotation (roll, pitch, yaw)
    return transform.block<3, 1>(0, 3);
}

bool LidarOdometry::detect_stationary_robot(const Eigen::Vector3d& linear_vel, const Eigen::Vector3d& angular_vel)
{
    // Check if both linear and angular velocities are below thresholds
    bool linear_stationary = linear_vel.norm() < velocity_threshold;
    bool angular_stationary = angular_vel.norm() < velocity_threshold;
    
    if (linear_stationary && angular_stationary) {
        stationary_count++;
        // Require multiple consecutive measurements to confirm stationary state
        // Increased threshold for more robust detection
        if (stationary_count >= 5) {
            is_robot_stationary = true;
        }
    } else {
        stationary_count = 0;
        is_robot_stationary = false;
    }
    
    return is_robot_stationary;
}

void LidarOdometry::compensate_for_terrain_inclination(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if (!has_imu_data || !latest_imu_ptr || cloud.points.size() < 10) {
        return;
    }
    
    // Use IMU linear acceleration to estimate gravity direction
    Eigen::Vector3d accel = latest_imu_ptr->linear_acceleration;
    if (accel.norm() < 0.1) {
        return; // Invalid acceleration data
    }
    
    // Gravity vector in sensor frame (negative of acceleration)
    Eigen::Vector3d gravity_sensor = -accel.normalized();
    
    // Desired gravity direction (world Z-axis pointing up)
    Eigen::Vector3d gravity_world(0, 0, 1);
    
    // Calculate rotation to align gravity vectors
    Eigen::Quaterniond correction_quat = Eigen::Quaterniond::FromTwoVectors(gravity_sensor, gravity_world);
    
    // Only apply correction if the inclination is significant (> 5 degrees)
    double angle = std::acos(std::abs(gravity_sensor.dot(gravity_world)));
    if (angle < 0.087) { // 5 degrees in radians
        return;
    }
    
    Eigen::Matrix3d correction_matrix = correction_quat.toRotationMatrix();
    
    // Apply correction to all points
    for (auto& point : cloud.points) {
        Eigen::Vector3d p(point.x, point.y, point.z);
        p = correction_matrix * p;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
    }
}

void LidarOdometry::remove_ground_points(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if (cloud.points.size() < 10) {
        return;
    }
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05); // 5cm threshold for better ground detection on slopes
    seg.setMaxIterations(50);
    seg.setProbability(0.99); // Higher confidence requirement
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    seg.setInputCloud(cloud_ptr);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.size() > cloud.points.size() * 0.3) { // If >30% are ground points
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(true); // Remove ground points
        extract.filter(cloud);
        
        // Ensure proper point cloud structure after filtering
        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = false;
    }
}

Eigen::Vector3d LidarOdometry::estimate_ground_plane_normal(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if (cloud.points.size() < 10) {
        return Eigen::Vector3d(0, 0, 1); // Default to Z-up
    }
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    seg.setInputCloud(cloud_ptr);
    seg.segment(*inliers, *coefficients);
    
    if (coefficients->values.size() >= 4) {
        return Eigen::Vector3d(coefficients->values[0], coefficients->values[1], coefficients->values[2]).normalized();
    }
    
    return Eigen::Vector3d(0, 0, 1);
}

void LidarOdometry::filter_points_by_height(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if (cloud.points.size() < 10) {
        return;
    }
    
    // Adaptive height filtering based on tilt state
    // When tilted, ground appears at different heights, so we need different thresholds
    double min_height, max_height;
    
    if (is_robot_tilted) {
        // More aggressive filtering when tilted to remove ground-as-wall artifacts
        min_height = -0.7;  // Deeper threshold to catch tilted ground
        max_height = 2.5;   // Lower ceiling to avoid false walls from distant ground
    } else {
        // Standard filtering for level operation
        min_height = -0.5;  // 50cm below sensor
        max_height = 3.0;   // 3m above sensor
    }
    
    auto it = std::remove_if(cloud.points.begin(), cloud.points.end(),
        [min_height, max_height, this](const pcl::PointXYZ& point) {
            // Basic height and validity checks
            if (point.z < min_height || point.z > max_height || 
                !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                return true;
            }
            
            // Additional filtering for tilted scenarios
            if (is_robot_tilted) {
                double distance = sqrt(point.x * point.x + point.y * point.y);
                // Remove points that form suspicious "walls" close to the robot when tilted
                if (distance < 1.0 && point.z > -0.2 && point.z < 0.3 && 
                    (std::abs(point.x) > 0.6 || std::abs(point.y) > 0.6)) {
                    return true; // Likely ground seen as wall due to tilt
                }
            }
            
            return false;
        });
    
    cloud.points.erase(it, cloud.points.end());
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;
}

void LidarOdometry::predict_motion_from_imu(const ImuDataPtr& current_imu, const ImuDataPtr& previous_imu)
{
    double dt = current_imu->timestamp - previous_imu->timestamp;
    if (dt <= 0 || dt > 0.5) return; // Skip invalid time deltas
    
    // Predict yaw change from angular velocity (Z-axis rotation for 2D)
    double angular_vel_z = current_imu->angular_velocity.z();
    predicted_yaw = angular_vel_z * dt;
    
    // Predict translation from current velocity estimate if available
    if (has_lidar_data) {
        predicted_translation = state_ptr->velocity.block<3, 1>(0, 0) * dt;
    } else {
        predicted_translation = Eigen::Vector3d::Zero();
    }
}

Eigen::Matrix4d LidarOdometry::create_initial_guess(double dx, double dy, double dyaw)
{
    Eigen::Matrix4d guess = Eigen::Matrix4d::Identity();
    
    // Set translation
    guess(0, 3) = dx;
    guess(1, 3) = dy;
    
    // Set rotation (yaw only for 2D)
    double c = cos(dyaw);
    double s = sin(dyaw);
    guess(0, 0) = c; guess(0, 1) = -s;
    guess(1, 0) = s; guess(1, 1) = c;
    
    return guess;
}

PoseEstimate LidarOdometry::fuse_pose_estimates(const PoseEstimate& lidar_est, const PoseEstimate& imu_est)
{
    PoseEstimate fused;
    fused.timestamp = lidar_est.timestamp;
    
    
    // Weight estimates by confidence - higher IMU weight for rotation
    double lidar_weight = lidar_est.confidence;
    double imu_weight = has_previous_imu ? 0.5 : 0.0; // Higher IMU weight for better rotation tracking
    double total_weight = lidar_weight + imu_weight;
    
    if (total_weight > 0) {
        // Weighted fusion of translation (LiDAR dominant)
        fused.translation = (lidar_weight * lidar_est.translation + imu_weight * imu_est.translation) / total_weight;
        
        // For yaw, heavily prioritize IMU for better rotation responsiveness
        if (lidar_est.confidence > 0.3) {
            fused.yaw = 0.2 * lidar_est.yaw + 0.8 * predicted_yaw; // Heavily IMU-dominant
        } else {
            fused.yaw = predicted_yaw; // Use IMU when LiDAR is unreliable
        }
        
        double base_confidence = lidar_weight / total_weight;
        fused.confidence = std::min(0.99, base_confidence * 1.2);
    } else {
        fused = lidar_est;
    }
    
    
    return fused;
}

double LidarOdometry::extract_yaw_from_transform(const Eigen::Matrix4d& transform)
{
    // Extract yaw angle from 2D rotation matrix
    return atan2(transform(1, 0), transform(0, 0));
}

void LidarOdometry::integrate_imu_motion(const ImuDataPtr& imu_data, double dt)
{
    if (dt <= 0 || dt > 0.5) return;
    
    // Update IMU pose estimate with integrated motion
    imu_pose_estimate.translation = predicted_translation;
    imu_pose_estimate.yaw = predicted_yaw;
    imu_pose_estimate.timestamp = imu_data->timestamp;
    imu_pose_estimate.confidence = 0.3; // Lower confidence than LiDAR
}

bool LidarOdometry::detect_excessive_tilt(const ImuDataPtr& imu_data)
{
    // Extract roll and pitch from IMU orientation quaternion
    Eigen::Vector3d euler = imu_data->orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    double roll = euler[0];
    double pitch = euler[1];
    
    // Check if roll or pitch exceeds maximum tilt angle
    bool tilted = (std::abs(roll) > max_tilt_angle_rad) || (std::abs(pitch) > max_tilt_angle_rad);
    
    is_robot_tilted = tilted;
    return tilted;
}

bool LidarOdometry::is_seeing_mostly_ground(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if (cloud.points.size() < 10) return false;
    
    int ground_points = 0;
    // Adaptive ground threshold based on tilt state
    double ground_z_threshold = is_robot_tilted ? -0.5 : -0.3; // More conservative when tilted
    const double max_ground_distance = 2.0; // Only check nearby points
    
    // When tilted, also check for points that appear as "walls" due to ground perspective
    for (const auto& point : cloud.points) {
        double distance = sqrt(point.x * point.x + point.y * point.y);
        if (distance < max_ground_distance) {
            // Standard ground detection
            if (point.z < ground_z_threshold) {
                ground_points++;
            }
            // Additional check for tilted scenarios: points forming vertical "walls" from ground
            else if (is_robot_tilted && distance < 1.5 && 
                     point.z > -0.1 && point.z < 0.2 && 
                     std::abs(point.x) > 0.5) { // Likely ground seen as wall
                ground_points++;
            }
        }
    }
    
    double ground_ratio = static_cast<double>(ground_points) / cloud.points.size();
    // More strict threshold when tilted to avoid false ground detection
    double threshold = is_robot_tilted ? ground_detection_threshold * 0.6 : ground_detection_threshold;
    return ground_ratio > threshold;
}

void LidarOdometry::enhanced_ground_plane_removal(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if (cloud.points.size() < 10) return;
    
    // Multiple passes with different parameters for better ground removal
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    
    // Adaptive parameters based on tilt state
    if (is_robot_tilted) {
        seg.setDistanceThreshold(0.08); // More permissive for tilted ground planes
        seg.setMaxIterations(150); // More iterations for challenging scenarios
    } else {
        seg.setDistanceThreshold(0.05); // Standard threshold for level operation
        seg.setMaxIterations(100);
    }
    seg.setProbability(0.99);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    
    // Perform multiple ground plane removals
    int max_passes = is_robot_tilted ? 3 : 2; // More passes when tilted
    for (int i = 0; i < max_passes && cloud_ptr->points.size() > 20; ++i) {
        seg.setInputCloud(cloud_ptr);
        seg.segment(*inliers, *coefficients);
        
        // Check if detected plane is likely ground
        if (inliers->indices.size() > cloud_ptr->points.size() * 0.15 && 
            coefficients->values.size() >= 4) {
            
            Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            normal.normalize();
            
            // Adaptive normal validation based on tilt state
            double normal_threshold;
            if (is_robot_tilted) {
                // When tilted, ground plane normal may not be perfectly vertical
                normal_threshold = 0.6; // More permissive for tilted scenarios
            } else {
                normal_threshold = 0.8; // Standard vertical threshold
            }
            
            // Check if plane normal indicates ground-like surface
            if (std::abs(normal.z()) > normal_threshold) {
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_ptr);
                extract.setIndices(inliers);
                extract.setNegative(true);
                extract.filter(*cloud_ptr);
                
            }
        }
    }
    
    cloud = *cloud_ptr;
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;
}

bool LidarOdometry::has_sufficient_features(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    // Check if cloud has enough points and feature diversity - lower threshold
    if (cloud.points.size() < 15) return false;
    
    // Check for point distribution (not all clustered in one area)
    double x_min = DBL_MAX, x_max = -DBL_MAX;
    double y_min = DBL_MAX, y_max = -DBL_MAX;
    
    for (const auto& point : cloud.points) {
        x_min = std::min(x_min, static_cast<double>(point.x));
        x_max = std::max(x_max, static_cast<double>(point.x));
        y_min = std::min(y_min, static_cast<double>(point.y));
        y_max = std::max(y_max, static_cast<double>(point.y));
    }
    
    double x_range = x_max - x_min;
    double y_range = y_max - y_min;
    
    // Require minimum spread in both directions - more permissive
    return (x_range > 0.3 && y_range > 0.3);
}

Eigen::Matrix3d LidarOdometry::get_tilt_compensation_matrix(const ImuDataPtr& imu_data)
{
    // Get current orientation from IMU
    Eigen::Matrix3d current_rotation = imu_data->orientation.toRotationMatrix();
    
    // Extract roll and pitch (ignore yaw for this compensation)
    Eigen::Vector3d euler = current_rotation.eulerAngles(0, 1, 2);
    double roll = euler[0];
    double pitch = euler[1];
    
    // Create compensation rotation to level the LiDAR horizontally
    Eigen::AngleAxisd roll_correction(-roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_correction(-pitch, Eigen::Vector3d::UnitY());
    
    return pitch_correction.toRotationMatrix() * roll_correction.toRotationMatrix();
}

void LidarOdometry::apply_tilt_compensation_to_cloud(pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Matrix3d& compensation)
{
    // Apply rotation compensation to all points
    for (auto& point : cloud.points) {
        Eigen::Vector3d p(point.x, point.y, point.z);
        p = compensation * p;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
    }
}

void LidarOdometry::update_pose_with_imu_only()
{
    if (!has_imu_data || !latest_imu_ptr || !has_previous_imu) return;
    
    // Use pure IMU integration when LiDAR is unreliable
    // Only apply yaw rotation, avoid translation drift
    Eigen::Vector3d translation_increment = predicted_translation;
    
    // Lighter IMU filtering for better responsiveness
    if (translation_increment.norm() < 0.001) {
        translation_increment = Eigen::Vector3d::Zero();
    } else if (translation_increment.norm() < 0.005) {
        translation_increment *= 0.8; // Reduce small IMU translations by 20%
    }
    
    
    // Constrain Z-axis movement
    translation_increment.z() = 0.0;
    
    POSE_G_L.block<3, 1>(0, 3) += translation_increment;
    
    // Force Z position to stay at ground level
    POSE_G_L(2, 3) = 0.0;
    
    // Apply yaw rotation from IMU
    Eigen::AngleAxisd yaw_rotation(predicted_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotation_matrix = yaw_rotation.toRotationMatrix();
    POSE_G_L.block<3, 3>(0, 0) = POSE_G_L.block<3, 3>(0, 0) * rotation_matrix;
    
    // Update state from pose matrix
    state_ptr->pose.translation() = POSE_G_L.block<3, 1>(0, 3);
    state_ptr->pose.linear() = POSE_G_L.block<3, 3>(0, 0);
    state_ptr->timestamp = latest_imu_ptr->timestamp;
}

bool LidarOdometry::validate_pose_estimate(const PoseEstimate& estimate, double dt)
{
    if (dt <= 0 || dt > 0.5) return false;
    
    // Calculate implied velocities
    double linear_velocity = estimate.translation.norm() / dt;
    double angular_velocity = std::abs(estimate.yaw) / dt;
    
    // Check for reasonable velocity limits (robot can't move too fast)
    const double max_linear_velocity = 2.0;  // 2 m/s maximum
    const double max_angular_velocity = 2.0; // 2 rad/s maximum
    
    if (linear_velocity > max_linear_velocity || angular_velocity > max_angular_velocity) {
        return false;
    }
    
    // Check for minimum confidence threshold
    if (estimate.confidence < 0.3) {
        return false;
    }
    
    // Additional validation: check for NaN or infinite values
    if (!estimate.translation.allFinite() || !std::isfinite(estimate.yaw)) {
        return false;
    }
    
    return true;
}

// ============= LOOP CLOSURE AND KEYFRAME METHODS =============

std::vector<float> LidarOdometry::compute_scan_context(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    std::vector<float> scan_context(SC_RING_NUM * SC_SECTOR_NUM, 0.0f);
    
    if (cloud.points.empty()) {
        return scan_context;
    }
    
    for (const auto& point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        
        // Convert to polar coordinates
        double distance = sqrt(point.x * point.x + point.y * point.y);
        double angle = atan2(point.y, point.x);
        
        // Skip points too close or too far
        if (distance < 0.5 || distance > SC_MAX_RADIUS) {
            continue;
        }
        
        // Compute ring and sector indices
        int ring_idx = static_cast<int>((distance / SC_MAX_RADIUS) * SC_RING_NUM);
        int sector_idx = static_cast<int>(((angle + M_PI) / (2.0 * M_PI)) * SC_SECTOR_NUM);
        
        // Clamp indices
        ring_idx = std::max(0, std::min(ring_idx, SC_RING_NUM - 1));
        sector_idx = std::max(0, std::min(sector_idx, SC_SECTOR_NUM - 1));
        
        // Accumulate height information (Z-coordinate represents structure)
        int idx = ring_idx * SC_SECTOR_NUM + sector_idx;
        scan_context[idx] = std::max(scan_context[idx], static_cast<float>(point.z + 2.0)); // Offset for positive values
    }
    
    // Normalize scan context
    float max_val = *std::max_element(scan_context.begin(), scan_context.end());
    if (max_val > 0) {
        for (auto& val : scan_context) {
            val /= max_val;
        }
    }
    
    return scan_context;
}

double LidarOdometry::compute_scan_context_distance(const std::vector<float>& sc1, const std::vector<float>& sc2)
{
    if (sc1.size() != sc2.size() || sc1.size() != SC_RING_NUM * SC_SECTOR_NUM) {
        return 1.0; // Maximum distance for invalid contexts
    }
    
    double min_distance = 1.0;
    
    // Try different sector shifts to account for rotation invariance
    for (int shift = 0; shift < SC_SECTOR_NUM; shift += 2) { // Skip every other to reduce computation
        double distance = 0.0;
        int valid_comparisons = 0;
        
        for (int ring = 0; ring < SC_RING_NUM; ++ring) {
            for (int sector = 0; sector < SC_SECTOR_NUM; ++sector) {
                int idx1 = ring * SC_SECTOR_NUM + sector;
                int shifted_sector = (sector + shift) % SC_SECTOR_NUM;
                int idx2 = ring * SC_SECTOR_NUM + shifted_sector;
                
                if (sc1[idx1] > 0.01 || sc2[idx2] > 0.01) { // Only compare meaningful values
                    distance += std::abs(sc1[idx1] - sc2[idx2]);
                    valid_comparisons++;
                }
            }
        }
        
        if (valid_comparisons > 0) {
            distance /= valid_comparisons;
            min_distance = std::min(min_distance, distance);
        }
    }
    
    return min_distance;
}

bool LidarOdometry::should_create_keyframe(const Eigen::Isometry3d& current_pose)
{
    if (keyframes.empty()) {
        return true; // Create first keyframe
    }
    
    // Check distance from last keyframe
    const Keyframe& last_kf = keyframes.back();
    Eigen::Vector3d translation_diff = current_pose.translation() - last_kf.pose.translation();
    double distance = translation_diff.norm();
    
    // Check rotation difference
    Eigen::Matrix3d rotation_diff = current_pose.rotation().transpose() * last_kf.pose.rotation();
    double angle_diff = std::abs(Eigen::AngleAxisd(rotation_diff).angle());
    
    return (distance > keyframe_distance_threshold) || (angle_diff > keyframe_angle_threshold);
}

void LidarOdometry::create_keyframe(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& pose, double timestamp)
{
    Keyframe kf;
    kf.cloud = cloud;
    kf.pose = pose;
    kf.scan_context = compute_scan_context(cloud);
    kf.timestamp = timestamp;
    kf.id = ++last_keyframe_id;
    
    keyframes.push_back(kf);
    
    // Add to pose graph
    add_pose_node(kf.id, pose, kf.id == 0); // Fix first keyframe
    
    // Add sequential edge to previous keyframe
    if (keyframes.size() > 1) {
        const Keyframe& prev_kf = keyframes[keyframes.size() - 2];
        Eigen::Matrix4d relative_transform = prev_kf.pose.inverse().matrix() * pose.matrix();
        add_pose_edge(prev_kf.id, kf.id, relative_transform, false);
    }
    
    // Limit keyframe history to prevent memory issues
    if (keyframes.size() > 1000) {
        keyframes.erase(keyframes.begin());
    }
}

std::vector<int> LidarOdometry::detect_loop_closure_candidates(const std::vector<float>& current_sc)
{
    std::vector<int> candidates;
    
    if (keyframes.size() < static_cast<size_t>(min_keyframes_between_loops)) {
        return candidates;
    }
    
    // Don't check recent keyframes (to avoid sequential matches)
    int end_idx = keyframes.size() - min_keyframes_between_loops;
    
    for (int i = 0; i < end_idx; ++i) {
        double distance = compute_scan_context_distance(current_sc, keyframes[i].scan_context);
        
        if (distance < loop_closure_threshold) {
            candidates.push_back(keyframes[i].id);
        }
    }
    
    // Sort candidates by similarity (lower distance = higher similarity)
    std::sort(candidates.begin(), candidates.end(), [&](int a, int b) {
        auto it_a = std::find_if(keyframes.begin(), keyframes.end(), [a](const Keyframe& kf) { return kf.id == a; });
        auto it_b = std::find_if(keyframes.begin(), keyframes.end(), [b](const Keyframe& kf) { return kf.id == b; });
        
        if (it_a != keyframes.end() && it_b != keyframes.end()) {
            double dist_a = compute_scan_context_distance(current_sc, it_a->scan_context);
            double dist_b = compute_scan_context_distance(current_sc, it_b->scan_context);
            return dist_a < dist_b;
        }
        return false;
    });
    
    // Return only top 3 candidates
    if (candidates.size() > 3) {
        candidates.resize(3);
    }
    
    return candidates;
}

bool LidarOdometry::verify_loop_closure(int candidate_id, const pcl::PointCloud<pcl::PointXYZ>& current_cloud, Eigen::Matrix4d& relative_transform)
{
    // Find candidate keyframe
    auto it = std::find_if(keyframes.begin(), keyframes.end(), 
                          [candidate_id](const Keyframe& kf) { return kf.id == candidate_id; });
    
    if (it == keyframes.end()) {
        return false;
    }
    
    const Keyframe& candidate_kf = *it;
    
    // Perform GICP registration between current scan and candidate keyframe
    pcl::PointCloud<pcl::PointXYZ> current_copy = current_cloud;
    pcl::PointCloud<pcl::PointXYZ> candidate_copy = candidate_kf.cloud;
    
    // Filter clouds for better registration
    enhanced_ground_plane_removal(current_copy);
    enhanced_ground_plane_removal(candidate_copy);
    filter_points_by_height(current_copy);
    filter_points_by_height(candidate_copy);
    
    if (current_copy.points.size() < 20 || candidate_copy.points.size() < 20) {
        return false;
    }
    
    // Create shared pointers for GICP
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(current_copy);
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(candidate_copy);
    
    // Setup GICP with more permissive parameters for loop closure
    auto loop_gicp = std::make_shared<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
    loop_gicp->setMaxCorrespondenceDistance(2.0); // More permissive for loop closure
    loop_gicp->setTransformationEpsilon(0.01);
    loop_gicp->setMaximumIterations(100);
    loop_gicp->setRANSACIterations(100);
    loop_gicp->setRANSACOutlierRejectionThreshold(0.2);
    
    loop_gicp->setInputSource(current_ptr);
    loop_gicp->setInputTarget(candidate_ptr);
    
    pcl::PointCloud<pcl::PointXYZ> aligned;
    loop_gicp->align(aligned);
    
    // Check registration quality
    if (!loop_gicp->hasConverged()) {
        return false;
    }
    
    double fitness_score = loop_gicp->getFitnessScore();
    if (fitness_score > 0.5) { // More permissive threshold for loop closure
        return false;
    }
    
    relative_transform = loop_gicp->getFinalTransformation().cast<double>();
    return true;
}

void LidarOdometry::perform_loop_closure(int matched_keyframe_id, const Eigen::Matrix4d& relative_transform)
{
    if (keyframes.empty()) {
        return;
    }
    
    int current_keyframe_id = keyframes.back().id;
    
    // Add loop closure edge to pose graph
    add_pose_edge(current_keyframe_id, matched_keyframe_id, relative_transform, true);
    
    // Mark pose graph for optimization
    pose_graph_needs_optimization = true;
}

// ============= POSE GRAPH OPTIMIZATION METHODS =============

void LidarOdometry::add_pose_node(int id, const Eigen::Isometry3d& pose, bool is_fixed)
{
    PoseNode node;
    node.pose = pose;
    node.is_fixed = is_fixed;
    pose_graph_nodes[id] = node;
}

void LidarOdometry::add_pose_edge(int from_id, int to_id, const Eigen::Matrix4d& relative_transform, bool is_loop_closure)
{
    PoseEdge edge;
    edge.from_id = from_id;
    edge.to_id = to_id;
    edge.relative_transform = relative_transform;
    edge.information_matrix = compute_information_matrix(relative_transform, is_loop_closure);
    edge.is_loop_closure = is_loop_closure;
    
    pose_graph_edges.push_back(edge);
}

void LidarOdometry::optimize_pose_graph()
{
    // Simple iterative optimization (Gauss-Newton approach)
    const int max_iterations = 10;
    const double convergence_threshold = 1e-6;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        double total_error = 0.0;
        std::map<int, Vector6d> delta_poses;
        
        // Initialize delta poses
        for (auto& [id, node] : pose_graph_nodes) {
            if (!node.is_fixed) {
                delta_poses[id] = Vector6d::Zero();
            }
        }
        
        // Compute residuals and Jacobians for each edge
        for (const auto& edge : pose_graph_edges) {
            if (pose_graph_nodes.find(edge.from_id) == pose_graph_nodes.end() ||
                pose_graph_nodes.find(edge.to_id) == pose_graph_nodes.end()) {
                continue;
            }
            
            const PoseNode& from_node = pose_graph_nodes[edge.from_id];
            const PoseNode& to_node = pose_graph_nodes[edge.to_id];
            
            // Compute predicted relative transform
            Eigen::Matrix4d predicted_transform = from_node.pose.inverse().matrix() * to_node.pose.matrix();
            
            // Compute error (simplified SE(3) error)
            Eigen::Matrix4d error_transform = edge.relative_transform.inverse() * predicted_transform;
            Eigen::Vector3d translation_error = error_transform.block<3, 1>(0, 3);
            Eigen::Matrix3d rotation_error = error_transform.block<3, 3>(0, 0);
            
            // Convert rotation error to axis-angle
            Eigen::AngleAxisd angle_axis(rotation_error);
            Eigen::Vector3d rotation_error_vec = angle_axis.angle() * angle_axis.axis();
            
            Vector6d error;
            error.head<3>() = translation_error;
            error.tail<3>() = rotation_error_vec;
            
            double error_norm = error.norm();
            total_error += error_norm;
            
            // Simple gradient descent update (simplified Jacobian)
            double learning_rate = edge.is_loop_closure ? 0.1 : 0.3;
            
            if (!from_node.is_fixed && delta_poses.find(edge.from_id) != delta_poses.end()) {
                delta_poses[edge.from_id] -= learning_rate * error;
            }
            if (!to_node.is_fixed && delta_poses.find(edge.to_id) != delta_poses.end()) {
                delta_poses[edge.to_id] += learning_rate * error;
            }
        }
        
        // Apply updates
        for (auto& [id, delta] : delta_poses) {
            if (pose_graph_nodes.find(id) != pose_graph_nodes.end()) {
                PoseNode& node = pose_graph_nodes[id];
                
                // Update translation
                node.pose.translation() += delta.head(3);
                
                // Update rotation
                Eigen::Vector3d rotation_delta = delta.tail(3);
                if (rotation_delta.norm() > 0.001) {
                    Eigen::AngleAxisd delta_rotation(rotation_delta.norm(), rotation_delta.normalized());
                    node.pose.rotate(delta_rotation);
                }
            }
        }
        
        // Check convergence
        if (total_error < convergence_threshold) {
            break;
        }
    }
}

Eigen::Matrix<double, 6, 6> LidarOdometry::compute_information_matrix(const Eigen::Matrix4d& /* transform */, bool is_loop_closure)
{
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
    
    if (is_loop_closure) {
        // Lower confidence for loop closures
        information *= 0.1;
    } else {
        // Higher confidence for sequential odometry
        information *= 1.0;
    }
    
    return information;
}

void LidarOdometry::update_poses_from_graph()
{
    // Update current robot pose from the latest keyframe in the graph
    if (!keyframes.empty()) {
        int latest_keyframe_id = keyframes.back().id;
        if (pose_graph_nodes.find(latest_keyframe_id) != pose_graph_nodes.end()) {
            const PoseNode& latest_node = pose_graph_nodes[latest_keyframe_id];
            
            // Update global pose
            POSE_G_L = latest_node.pose.matrix();
            state_ptr->pose = latest_node.pose;
            
            // Update keyframe poses in memory
            for (auto& kf : keyframes) {
                if (pose_graph_nodes.find(kf.id) != pose_graph_nodes.end()) {
                    kf.pose = pose_graph_nodes[kf.id].pose;
                }
            }
        }
    }
}
