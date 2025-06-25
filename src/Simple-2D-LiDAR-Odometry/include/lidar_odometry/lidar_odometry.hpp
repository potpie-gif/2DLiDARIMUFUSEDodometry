#ifndef LIDAR_ODOMETRY_H
#define LIDAR_ODOMETRY_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/registration/gicp.h>
#include "lidar_odometry/utils.hpp"

class LidarOdometry
{
    public:
        LidarOdometry(double max_correspondence_distance = 1.0, double transformation_epsilon = 0.001, double maximum_iterations = 1000);
        StatePtr get_state();
        void process_scan_data(const ScanDataPtr scan_data);
        void process_imu_data(const ImuDataPtr imu_data);
        void update_pose_with_fusion();
    private:

        ScanDataPtr last_scan_ptr;
        ImuDataPtr latest_imu_ptr;
        StatePtr state_ptr;
        Eigen::Matrix4d POSE_G_L; // pose in SE(3) (ground -> LiDAR)

        // Separate translation and rotation components
        Eigen::Vector3d lidar_translation;
        Eigen::Quaterniond imu_orientation;
        
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr gicp;
        
        bool has_imu_data;
        bool has_lidar_data;

        Eigen::Matrix4d get_transform_matrix(ScanDataPtr source, ScanDataPtr target);
        Eigen::Vector3d extract_translation_only(const Eigen::Matrix4d& transform);
};

#endif
