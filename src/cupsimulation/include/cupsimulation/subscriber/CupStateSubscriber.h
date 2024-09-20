/**
 * @file CupStateSubscriber.h
 * @brief Defines the CupStateSubscriber class, which is responsible for subscribing to cup state information in a ROS 2 system.
 * @author SHG Selten
 * @version 0.1
 * @date 2023-04-04
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/**
 * @brief A class representing a subscriber for cup state information.
 * 
 * This class inherits from the rclcpp::Node class and provides functionality to subscribe to cup state information,
 * perform periodic publishing, and check if a cup is in grip.
 */
class CupStateSubscriber : public rclcpp::Node
{

public:
    /**
     * @brief Default constructor for CupStateSubscriber.
     */
    CupStateSubscriber();

    /**
     * @brief Default destructor for CupStateSubscriber.
     */
    ~CupStateSubscriber();

    /**
     * @brief Function triggered by the timer for periodic publishing.
     */
    void timerTriggered();

private:
    /**
     * @brief Flag indicating whether debug mode is enabled.
     */
    bool _debug = false;

    /**
     * @brief Positive margin for cup pickup.
     *
     * This variable represents the positive margin used for cup pickup.
     * It is used to determine the acceptable range for detecting a cup pickup event.
     */
    double _positivePickupMarginX = 0.07; 
    double _positivePickupMarginY = -0.09;
    double _positivePickupMarginZ = 0.04;

    /**
     * @brief Negative margin for cup pickup.
     * 
     * This variable represents the negative margin used for cup pickup. It is used to determine the threshold
     * below which a cup is considered to be picked up.
     */
    double _negativePickupMarginX =  0.05; 
    double _negativePickupMarginY = -0.11;
    double _negativePickupMarginZ = -0.03;

    /**
     * @brief A shared pointer to a TimerBase object.
     *
     * This shared pointer is used to manage the lifetime of a TimerBase object.
     * It allows multiple objects to share ownership of the same TimerBase instance.
     * When the last shared pointer to the TimerBase object is destroyed, the TimerBase object is deleted.
     */
    rclcpp::TimerBase::SharedPtr _timer;

    /**
     * @brief A shared pointer to a `tf2_ros::TransformListener` object.
     * 
     * This shared pointer is used as a transform listener for tf2.
     */
    /**< Transform listener for tf2. */
    std::shared_ptr<tf2_ros::TransformListener> _tf2Listener; 

    /**
     * @brief Shared pointer to a publisher for cup state information.
     *
     * This shared pointer is used to publish cup state information using the `rclcpp::Publisher` class.
     * It is templated with the message type `geometry_msgs::msg::TransformStamped`.
     */
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr _cupStatePublisher; 
    
    /**
     * @brief A shared pointer to a tf2_ros::Buffer object.
     *
     * This shared pointer is used as a buffer for tf2 transformations.
     */
    std::shared_ptr<tf2_ros::Buffer> _tf2Buffer;

    /**
     * @brief Checks if a cup is in grip based on the provided transform.
     * 
     * @param transform The transform of the cup.
     * @return True if the cup is in grip, false otherwise.
     */
    bool isCupInGrip(const geometry_msgs::msg::TransformStamped& transform);
};
