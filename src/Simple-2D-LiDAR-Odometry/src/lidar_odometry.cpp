
#include "lidar_odometry/lidar_odometry.hpp"

LidarOdometry::LidarOdometry(double max_correspondence_distance, double transformation_epsilon, double maximum_iterations)
{
    gicp = std::make_shared<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();

    gicp->setMaxCorrespondenceDistance(max_correspondence_distance);
    gicp->setTransformationEpsilon(transformation_epsilon);
    gicp->setMaximumIterations(maximum_iterations);

    // Initialize pose components separately
    lidar_translation = Eigen::Vector3d::Zero();
    imu_orientation = Eigen::Quaterniond::Identity();
    
    has_imu_data = false;
    has_lidar_data = false;

    POSE_G_L = Eigen::Matrix4d::Identity();

    state_ptr = std::make_shared<State>();
    state_ptr->pose = Eigen::Isometry3d::Identity();
    state_ptr->velocity = Eigen::Matrix<double, 6, 1>::Zero();
}

StatePtr LidarOdometry::get_state()
{
    return state_ptr;
}

void LidarOdometry::process_scan_data(ScanDataPtr scan_ptr)
{
    if (last_scan_ptr) {
        double dt = scan_ptr->timestamp - last_scan_ptr->timestamp;
        Eigen::Matrix4d transform_matrix = get_transform_matrix(last_scan_ptr, scan_ptr);
        
        // Extract only translation from LIDAR
        Eigen::Vector3d delta_translation = extract_translation_only(transform_matrix);
        lidar_translation += delta_translation;
        
        // Calculate translation velocity
        state_ptr->velocity.block<3, 1>(0, 0) = delta_translation / dt;
        
        has_lidar_data = true;
        update_pose_with_fusion();
    }

    last_scan_ptr = scan_ptr;
}

void LidarOdometry::process_imu_data(const ImuDataPtr imu_data)
{
    static ImuDataPtr last_imu_ptr = nullptr;
    
    if (last_imu_ptr) {
        double dt = imu_data->timestamp - last_imu_ptr->timestamp;
        
        // Integrate angular velocity to get orientation change
        Eigen::Vector3d angular_velocity = imu_data->angular_velocity;
        
        // Simple integration - for 2D, we mainly care about Z-axis rotation
        double yaw_rate = angular_velocity.z();
        
        // Update orientation by integrating angular velocity
        Eigen::AngleAxisd rotation_update(yaw_rate * dt, Eigen::Vector3d::UnitZ());
        imu_orientation = imu_orientation * Eigen::Quaterniond(rotation_update);
        
        // Store angular velocity from IMU
        state_ptr->velocity.block<3, 1>(3, 0) = angular_velocity;
        
        has_imu_data = true;
        update_pose_with_fusion();
    }
    
    last_imu_ptr = imu_data;
}

void LidarOdometry::update_pose_with_fusion()
{
    if (has_lidar_data && has_imu_data) {
        // Combine LiDAR translation with IMU rotation
        state_ptr->pose.translation() = lidar_translation;
        state_ptr->pose.linear() = imu_orientation.toRotationMatrix();
        
        // Update the combined pose matrix
        POSE_G_L.block<3, 3>(0, 0) = state_ptr->pose.linear();
        POSE_G_L.block<3, 1>(0, 3) = state_ptr->pose.translation();
    }
}

Eigen::Matrix4d LidarOdometry::get_transform_matrix(ScanDataPtr source, ScanDataPtr target)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);

    gicp->setInputSource(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(source->point_cloud));
    gicp->setInputTarget(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(target->point_cloud));
    gicp->align(*align);

    Eigen::Matrix4f src2tgt = gicp->getFinalTransformation();

    return src2tgt.cast<double>();    
}

Eigen::Vector3d LidarOdometry::extract_translation_only(const Eigen::Matrix4d& transform)
{
    // Extract only the translation component, ignore rotation
    return transform.block<3, 1>(0, 3);
}
