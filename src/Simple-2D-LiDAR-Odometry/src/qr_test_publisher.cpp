#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class QrTestPublisher : public rclcpp::Node
{
public:
  QrTestPublisher() : Node("qr_test_publisher"), qr_counter_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/qr", 10);
    timer_ = this->create_wall_timer(
      10s, std::bind(&QrTestPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "QR test publisher started - publishing every 10 seconds");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "QR_CODE_" + std::to_string(qr_counter_++);
    
    RCLCPP_INFO(this->get_logger(), "Publishing QR code: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t qr_counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QrTestPublisher>());
  rclcpp::shutdown();
  return 0;
}