#include <rclcpp/rclcpp.hpp>
#include "CupStatePublisher.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CupStatePublisher>());
  rclcpp::shutdown();
  return 0;
}