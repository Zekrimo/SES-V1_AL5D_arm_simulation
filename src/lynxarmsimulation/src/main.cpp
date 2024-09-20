#include <rclcpp/rclcpp.hpp>
#include "../include/ArmStatePublisher.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmStatePublisher>());
  rclcpp::shutdown();
  return 0;
}