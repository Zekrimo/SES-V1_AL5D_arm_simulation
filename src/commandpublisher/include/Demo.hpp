/**
 * @file Demo.hpp
 * @brief Defines the Demo class, which is responsible for sending serial messages in a ROS 2 system.
 * @author SHG Selten
 * @version 0.1
 * @date 2023-04-04
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "SerialMessages.hpp"

/**
 * @brief The Demo class represents a ROS2 node for sending serial messages.
 */
class Demo : public rclcpp::Node
{
public:
    /**
     * @brief Constructs a Demo object.
     */
    Demo(std::string customArg);

    /**
     * @brief Destroys the Demo object.
     */
    ~Demo();

    void processSerialMessage();

private:
    /**
     * @brief Flag indicating whether debug mode is enabled or not.
     */
    bool _debug = false;

    bool _customMessageProvided = false;

    /**
     * @brief A class that represents a custom message.
     */
    std::string _customMessage;

    /**
     * @brief Sends a test message.
     */
    void sendTestMessage();

    /**
     * @brief Initializes the default pose.
     */
    void initDefaultPose();

    /**
     * @brief Sends a serial message.
     * @param message The message to be sent.
     */
    void sendSerialMessage(std::string message);

    /**
     * @brief A shared pointer to a publisher object that publishes messages of type std_msgs::msg::String.
     *
     * This shared pointer is used to manage the lifetime of the publisher object and provides access to its functionality.
     * It is typically used to publish messages to a ROS topic.
     */
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _serialMessagePublisher;

    /**
     * @brief A shared pointer to a TimerBase object in the rclcpp namespace.
     *
     * This shared pointer is used to manage the lifetime of a TimerBase object.
     * It allows multiple objects to share ownership of the TimerBase object and
     * automatically destroys the object when it is no longer needed.
     */
    rclcpp::TimerBase::SharedPtr _timer;
};
