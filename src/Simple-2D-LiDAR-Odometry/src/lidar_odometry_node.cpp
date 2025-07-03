#include <cstdio>
#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "lidar_odometry/lidar_odometry.hpp"

class LidarOdometryNode : public rclcpp::Node
{
public:
  LidarOdometryNode() : Node("lidar_odometry_node")
  {
    RCLCPP_INFO(this->get_logger(), "lidar_odometry_node");

    parameter_initialization();

    double max_correspondence_distance;
    double transformation_epsilon;
    double maximum_iterations;
    std::string scan_topic_name;
    std::string imu_topic_name;
    std::string odom_topic_name;
    double lidar_offset_x, lidar_offset_y, lidar_offset_z;
    double lidar_roll, lidar_pitch, lidar_yaw;

    this->get_parameter("max_correspondence_distance", max_correspondence_distance);
    this->get_parameter("transformation_epsilon", transformation_epsilon);
    this->get_parameter("maximum_iterations", maximum_iterations);
    this->get_parameter("scan_topic_name", scan_topic_name);
    this->get_parameter("imu_topic_name", imu_topic_name);
    this->get_parameter("odom_topic_name", odom_topic_name);
    this->get_parameter("lidar_offset_x", lidar_offset_x);
    this->get_parameter("lidar_offset_y", lidar_offset_y);
    this->get_parameter("lidar_offset_z", lidar_offset_z);
    this->get_parameter("lidar_roll", lidar_roll);
    this->get_parameter("lidar_pitch", lidar_pitch);
    this->get_parameter("lidar_yaw", lidar_yaw);

    RCLCPP_INFO(this->get_logger(), "===== Configuration =====");
    RCLCPP_INFO(this->get_logger(), "max_correspondence_distance: %.4f", max_correspondence_distance);
    RCLCPP_INFO(this->get_logger(), "transformation_epsilon: %.4f", transformation_epsilon);
    RCLCPP_INFO(this->get_logger(), "maximum_iterations %.4f", maximum_iterations);
    RCLCPP_INFO(this->get_logger(), "scan_topic_name: %s", scan_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic_name: %s", imu_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "odom_topic_name: %s", odom_topic_name.c_str());

    // Create LiDAR mounting transform from parameters
    lidar_mounting_transform_ = Eigen::Isometry3d::Identity();
    lidar_mounting_transform_.translation() = Eigen::Vector3d(lidar_offset_x, lidar_offset_y, lidar_offset_z);
    lidar_mounting_transform_.linear() = (Eigen::AngleAxisd(lidar_yaw, Eigen::Vector3d::UnitZ()) *
                                         Eigen::AngleAxisd(lidar_pitch, Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(lidar_roll, Eigen::Vector3d::UnitX())).matrix();

    lidar_odometry_ptr = std::make_shared<LidarOdometry>(
      max_correspondence_distance, transformation_epsilon, maximum_iterations);

    odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 100);
    marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("qr_markers", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    rclcpp::QoS scan_qos(10);
    scan_qos.best_effort();
    
    rclcpp::QoS imu_qos(100); // Larger queue for high-frequency IMU processing
    imu_qos.reliable(); // Use reliable for IMU to avoid drops

    scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_name, scan_qos,
      std::bind(&LidarOdometryNode::scan_callback, this, std::placeholders::_1)
    );

    imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_name, imu_qos,
      std::bind(&LidarOdometryNode::imu_callback, this, std::placeholders::_1)
    );

    qr_subscriber = this->create_subscription<std_msgs::msg::String>(
      "/qr", 10,
      std::bind(&LidarOdometryNode::qr_callback, this, std::placeholders::_1)
    );

    publish_static_transform();
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_subscriber;
  std::shared_ptr<LidarOdometry> lidar_odometry_ptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  
  std::vector<visualization_msgs::msg::Marker> qr_markers_;
  int marker_id_counter_ = 0;
  
  // LiDAR mounting offset transform
  Eigen::Isometry3d lidar_mounting_transform_;
  

  void parameter_initialization()
  {
    this->declare_parameter<double>("max_correspondence_distance", 1.0);
    this->declare_parameter<double>("transformation_epsilon", 0.005);
    this->declare_parameter<double>("maximum_iterations", 30);
    this->declare_parameter<std::string>("scan_topic_name", "ldlidar_node/scan");
    this->declare_parameter<std::string>("imu_topic_name", "/imu/mpu6050");
    this->declare_parameter<std::string>("odom_topic_name", "odom");
    
    // LiDAR mounting offset parameters
    this->declare_parameter<double>("lidar_offset_x", 0.0);
    this->declare_parameter<double>("lidar_offset_y", 0.0);
    this->declare_parameter<double>("lidar_offset_z", 0.0);
    this->declare_parameter<double>("lidar_roll", 0.0);
    this->declare_parameter<double>("lidar_pitch", 0.0);
    this->declare_parameter<double>("lidar_yaw", 0.0);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    auto point_cloud_msg = laser2cloudmsg(scan_msg);
    auto pcl_point_cloud = cloudmsg2cloud(point_cloud_msg);

    // Filter out robot body points
    auto filtered_cloud = filter_robot_body(pcl_point_cloud);


    if (filtered_cloud.size() < 20) {
        return;
    }

    auto scan_data = std::make_shared<ScanData>();
    scan_data->timestamp = scan_msg->header.stamp.sec + scan_msg->header.stamp.nanosec / 1e9;
    scan_data->point_cloud = filtered_cloud;

    lidar_odometry_ptr->process_scan_data(scan_data);
    publish_odometry();
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
  {
    auto imu_data = std::make_shared<ImuData>();
    imu_data->timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec / 1e9;
    
    // Convert ROS IMU message to our ImuData structure with inverted yaw rate
    imu_data->angular_velocity = Eigen::Vector3d(
      imu_msg->angular_velocity.x,
      imu_msg->angular_velocity.y,
      -imu_msg->angular_velocity.z  // Invert yaw rate for left/right flip
    );
    
    // Optimized orientation processing - avoid heavy quaternion operations
    if (imu_msg->orientation.w != 0.0 || imu_msg->orientation.x != 0.0 || 
        imu_msg->orientation.y != 0.0 || imu_msg->orientation.z != 0.0) {
      // Directly use quaternion with inverted Z component for yaw flip
      imu_data->orientation = Eigen::Quaterniond(
        imu_msg->orientation.w,
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        -imu_msg->orientation.z  // Simple Z inversion for yaw flip
      ).normalized();
    } else {
      // MPU6050 doesn't provide orientation, use identity for integration
      imu_data->orientation = Eigen::Quaterniond::Identity();
    }
    
    imu_data->linear_acceleration = Eigen::Vector3d(
      imu_msg->linear_acceleration.x,
      imu_msg->linear_acceleration.y,
      imu_msg->linear_acceleration.z
    );

    lidar_odometry_ptr->process_imu_data(imu_data);
    
    // Process all IMU data without rate limiting for responsive rotation tracking
    publish_odometry();
  }

  void publish_odometry()
  {
    auto state = lidar_odometry_ptr->get_state();
    
    // Only publish if we have valid state data
    if (!state) {
      return;
    }
    
    std::string fixed_id = "odom";
    std::string child_id = "base_link";

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = fixed_id;
    odom_msg.child_frame_id = child_id;
    odom_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(state->timestamp * 1e9));

    odom_msg.pose.pose = Eigen::toMsg(state->pose);
    odom_msg.twist.twist = Eigen::toMsg(state->velocity);

    odom_publisher->publish(odom_msg);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = odom_msg.header.stamp;
    odom_tf.header.frame_id = fixed_id;
    odom_tf.child_frame_id = child_id;
    odom_tf.transform.translation.x = state->pose.translation().x();
    odom_tf.transform.translation.y = state->pose.translation().y();
    odom_tf.transform.translation.z = state->pose.translation().z();
    odom_tf.transform.rotation = Eigen::toMsg(Eigen::Quaterniond(state->pose.rotation()));

    tf_broadcaster_->sendTransform(odom_tf);
  }

  void publish_static_transform()
  {
    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = this->get_clock()->now();
    static_transform.header.frame_id = "base_link";
    static_transform.child_frame_id = "ldlidar_base";
    
    // Use configured LiDAR mounting offset
    static_transform.transform.translation.x = lidar_mounting_transform_.translation().x();
    static_transform.transform.translation.y = lidar_mounting_transform_.translation().y();
    static_transform.transform.translation.z = lidar_mounting_transform_.translation().z();
    
    Eigen::Quaterniond q(lidar_mounting_transform_.rotation());
    static_transform.transform.rotation.x = q.x();
    static_transform.transform.rotation.y = q.y();
    static_transform.transform.rotation.z = q.z();
    static_transform.transform.rotation.w = q.w();

    static_broadcaster_->sendTransform(static_transform);
  }

  void qr_callback(const std_msgs::msg::String::SharedPtr qr_msg)
  {
    auto state = lidar_odometry_ptr->get_state();
    if (!state) {
      RCLCPP_WARN(this->get_logger(), "No valid odometry state available for QR marker");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "QR code detected: %s at position [%.3f, %.3f, %.3f]", 
                qr_msg->data.c_str(),
                state->pose.translation().x(),
                state->pose.translation().y(),
                state->pose.translation().z());

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "qr_codes";
    marker.id = marker_id_counter_++;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = state->pose.translation().x();
    marker.pose.position.y = state->pose.translation().y();
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.1;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.7;

    marker.lifetime = rclcpp::Duration::from_seconds(0);

    qr_markers_.push_back(marker);
    publish_qr_markers();
  }

  void publish_qr_markers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers = qr_markers_;
    marker_publisher->publish(marker_array);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
