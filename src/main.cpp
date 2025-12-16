#include <lidar_wheel_slam_demo/lidar_wheel_slam_demo_component.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<c107::LidarWheelSLAMDemo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}