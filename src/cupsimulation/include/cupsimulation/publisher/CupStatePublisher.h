/**
 * @file CupStatePublisher.h
 * @brief Defines the CupStatePublisher class, which is responsible for publishing the state of a cup in a ROS 2 system.
* @author SHG Selten
 * @version 0.1
 * @date 2023-04-04
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

/**
 * @brief Enumerates the possible states of a cup.
 */
enum State
{
    IDLE,       ///< The cup is idle.
    FALLING,    ///< The cup is falling.
    GRABBED     ///< The cup is grabbed.
}; 

/**
 * @class CupStatePublisher
 * @brief Publishes the state of a cup in a ROS 2 system.
 */
class CupStatePublisher : public rclcpp::Node 
{
public:

    /**
     * @brief Constructs a CupStatePublisher object.
     */
    CupStatePublisher(/* args */);

    /**
     * @brief Destroys the CupStatePublisher object.
     */
    ~CupStatePublisher();

    /**
     * @brief Publishes the state of the cup.
     */
    void publishCupState();

private:
    /**
     * @brief The falling speed of the cup.
     */
    double _fallSpeed;  

    /**
     * @brief The ground level of the cup.
     */
    double _groundLevel;

    /**
     * @brief The starting X position of the cup.
     */
    double _startXpos; 

    /**
     * @brief The starting Y position of the cup.
     */
    double _startYpos; 

    /**
     * @brief A smart pointer that owns and manages an object of type tf2_ros::TransformBroadcaster.
     * 
     * The ownership of the object is transferred to the unique_ptr and the object is destroyed when the unique_ptr goes out of scope.
     * 
     * @see tf2_ros::TransformBroadcaster
     */
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf2Broadcaster;  

    /**
     * @brief A shared pointer to a subscription object that subscribes to messages of type `geometry_msgs::msg::TransformStamped`.
     *
     * This shared pointer is used to manage the lifetime of the subscription object and provides access to its member functions and data.
     */
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr _cupSubscriber;  

    /**
     * @brief A shared pointer to a publisher object that publishes messages of type std_msgs::msg::String.
     */
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _cupDescriptionPublisher;
    
    /**
     * @brief A shared pointer to a timer object in the rclcpp library.
     * 
     * This shared pointer is used to manage the lifetime of a timer object.
     * It allows multiple objects to share ownership of the timer and automatically
     * deallocates the timer when the last shared pointer goes out of scope.
     */
    rclcpp::TimerBase::SharedPtr _timer;  

    /**
     * @brief A message representing a transformation between two coordinate frames.
     * 
     * This message is used to represent the transformation between two coordinate frames in a ROS system.
     * It contains the translation and rotation components of the transformation.
     */
    geometry_msgs::msg::TransformStamped _cupPoseMsg; 

    /**
     * Represents the state of the cup.
     */
    enum State _state;

    /**
     * @brief Initializes the cup's pose message.
     */
    void initCupPoseMsg();

    /**
     * @brief Handles the cup message received from the subscriber.
     * @param msg The cup message.
     */
    void handleCup(const geometry_msgs::msg::TransformStamped& msg);

    /**
     * @brief Sets the cup's pose message based on the received cup message.
     * @param msg The cup message.
     */
    void setCupPoseMsg(const geometry_msgs::msg::TransformStamped& msg);

    /**
     * @brief Applies gravitational force to the cup.
     */
    void affectCupByGravitationalForce();
};

