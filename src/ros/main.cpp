#include "cartesian_manager/ros/cartesian_manager.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ros_cartesian_manager::CartesianManagerROS>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
