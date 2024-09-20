#include <rclcpp/rclcpp.hpp>
#include "CupStateSubscriber.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CupStateSubscriber>());
  rclcpp::shutdown();
  return 0;
}