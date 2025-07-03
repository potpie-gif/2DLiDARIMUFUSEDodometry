#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <cfloat>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "laser_geometry/laser_geometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"


constexpr double D_R = M_PI / 180.;
constexpr double R_D = 180. / M_PI;
constexpr double g = 9.81007;

struct State {
    Eigen::Isometry3d pose;
    Eigen::Matrix<double ,6, 1> velocity;
    double timestamp;
};

using StatePtr = std::shared_ptr<State>;

struct ScanData {
    double timestamp;
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
};

using ScanDataPtr = std::shared_ptr<ScanData>;

struct ImuData {
    double timestamp;
    Eigen::Vector3d angular_velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d linear_acceleration;
};

struct PoseEstimate {
    Eigen::Vector3d translation;
    double yaw;
    double timestamp;
    double confidence;
};

using ImuDataPtr = std::shared_ptr<ImuData>;

inline pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::msg::PointCloud2 &cloudmsg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_dst;
    pcl::fromROSMsg(cloudmsg, cloud_dst);
    return cloud_dst;
}

inline sensor_msgs::msg::PointCloud2 laser2cloudmsg(sensor_msgs::msg::LaserScan::SharedPtr laser, std::string frame_id = "scan")
{
    static laser_geometry::LaserProjection projector;
    sensor_msgs::msg::PointCloud2 pc2_dst;
    projector.projectLaser(*laser, pc2_dst,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
    pc2_dst.header.frame_id = frame_id;

    return pc2_dst;
}

inline pcl::PointCloud<pcl::PointXYZ> filter_robot_body(const pcl::PointCloud<pcl::PointXYZ>& input_cloud)
{
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    
    // Filter out points that are likely from the robot body
    // Remove points 15cm in front and 20cm from either side
    double min_range = 0.05;  // Minimum range to ignore very close points
    double max_range = 0.6;   // Maximum range to limit to 0.6 meters
    double max_front_distance = 0.15;  // 15cm in front of robot
    double robot_half_width = 0.20;    // 20cm on each side
    
    for (const auto& point : input_cloud.points) {
        // Flip LiDAR front/back: X becomes -X (flips front/back)
        pcl::PointXYZ flipped_point;
        flipped_point.x = -point.x;  // Flip X-axis (front/back)
        flipped_point.y = point.y;   // Keep Y-axis (left/right)
        flipped_point.z = point.z;   // Keep Z-axis (up/down)
        
        double range = sqrt(flipped_point.x * flipped_point.x + flipped_point.y * flipped_point.y);
        
        // Keep points that are:
        // 1. Beyond minimum range
        // 2. Within maximum range (0.6 meters)
        // 3. Not in the robot body area (15cm front, 20cm each side)
        if (range > min_range && range <= max_range && !(flipped_point.y > 0 && flipped_point.y < max_front_distance && abs(flipped_point.x) < robot_half_width)) {
            filtered_cloud.points.push_back(flipped_point);
        }
    }
    
    filtered_cloud.width = filtered_cloud.points.size();
    filtered_cloud.height = 1;
    filtered_cloud.is_dense = true;
    
    return filtered_cloud;
}

inline Eigen::Matrix4d inverseSE3(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);

    // 역행렬 계산
    Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
    T_inv.block<3, 3>(0, 0) = R.transpose();
    T_inv.block<3, 1>(0, 3) = -R.transpose() * t;

    return T_inv;
}

#endif
