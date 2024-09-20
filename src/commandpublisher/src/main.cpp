#include <rclcpp/rclcpp.hpp>
#include "../include/Demo.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::string customArg = "false";
  // Check if an additional argument is passed
  if (argc > 1)
  {
    customArg = argv[1];
    std::cout << "Received custom argument: " << customArg << std::endl;
  }
  else
  {
    std::cout << "No custom argument provided." << std::endl;
  }

  auto node = std::make_shared<Demo>(customArg); 

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}