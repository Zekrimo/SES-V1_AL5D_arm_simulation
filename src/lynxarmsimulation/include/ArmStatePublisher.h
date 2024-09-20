/**
 * @file ArmStatePublisher.h
 * @brief Header file for the ArmStatePublisher class.
 * 
 * This file contains the declaration of the ArmStatePublisher class, which is responsible for publishing the state of the arm.
 * The class inherits from rclcpp::Node and provides functionality to publish the state of the arm using ROS 2 messages.
 * It also includes methods for building the arm's world, joints, cup, and arm itself.
 * 
 * @author SHG Selten
 * @version 0.1
 * @date 2023-04-04
 */
#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <urdf/model.h>
#include <cmath>


enum class JointNames {
    BASE_LINK2TURRET = 0,
    TURRET2UPPERARM = 1,
    UPPERARM2FOREARM = 2,
    FOREARM2WRIST = 3,
    WRIST2HAND = 4,
    GRIPPER_LEFT2HAND = 5,
    GRIPPER_RIGHT2HAND = 6,
} ;

/**
 * @brief Class responsible for publishing the state of the arm.
 * 
 * This class inherits from rclcpp::Node and provides functionality to publish the state of the arm
 * using ROS 2 messages. It also includes methods for building the arm's world, joints, cup, and arm itself.
 */
class ArmStatePublisher : public rclcpp::Node {
public:
    /**
     * @brief Constructor for ArmStatePublisher class.
     */
    ArmStatePublisher();

    /**
     * @brief Destructor for ArmStatePublisher class.
     */
    ~ArmStatePublisher();

    /**
     * @brief Publishes the state of the arm.
     * 
     * This method is triggered by a timer and publishes the state of the arm as a sensor_msgs::msg::JointState message.
     */
    void publishArmState();

private:

    /**
     * @brief The default time in milliseconds for something.
     */
    int defaultTimeMs = 10000;

    /**
     * @brief A message type used to represent the state of the robot arm's joints.
     * 
     * The `sensor_msgs::msg::JointState` message type provides information about the position, velocity, and effort of each joint in the robot arm.
     * It is commonly used in ROS (Robot Operating System) for publishing and subscribing to joint state information.
     */
    sensor_msgs::msg::JointState armStateMsg;

    /**
     * @brief Flag indicating whether debug mode is enabled or not.
     * 
     * When debug mode is enabled, additional debug information will be printed.
     * Set this flag to true for enabling debug mode, and false otherwise.
     */
    bool debug = false;

    bool emergencyStop = false;

    int maxDuration = 20000;


    /**
     * @brief A class for publishing static transforms.
     *
     * The `tf2_ros::StaticTransformBroadcaster` class provides a way to publish static transforms
     * that do not change over time. It is used to broadcast transformations between coordinate frames
     * in a ROS system.
     *
     * This class is part of the `tf2_ros` package, which is a ROS wrapper for the `tf2` library.
     * The `tf2` library provides support for managing coordinate frames and transformations in ROS.
     *
     * To use the `StaticTransformBroadcaster`, you need to create an instance of this class and call
     * its `sendTransform()` method to publish the static transform. The transform is represented by
     * a `geometry_msgs::TransformStamped` message, which contains the transformation information
     * including the source and target frames, translation, and rotation.
     *
     * Example usage:
     * ```
     * tf2_ros::StaticTransformBroadcaster broadcaster;
     * geometry_msgs::TransformStamped transformStamped;
     * // Set the transformation information
     * transformStamped.header.frame_id = "parent_frame";
     * transformStamped.child_frame_id = "child_frame";
     * transformStamped.transform.translation.x = 1.0;
     * transformStamped.transform.rotation.w = 1.0;
     * // Publish the static transform
     * broadcaster.sendTransform(transformStamped);
     * ```
     *
     * For more information on using the `tf2_ros::StaticTransformBroadcaster` class, refer to the
     * official ROS documentation.
     */
    tf2_ros::StaticTransformBroadcaster _tf2StaticBroadcaster_;

    /**
     * @brief A shared pointer to a publisher object that publishes messages of type sensor_msgs::msg::JointState.
     * 
     * This publisher sends parsed serial messages to rviz simulation.
     */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _armStatePublisher;

    /**
     * @brief A shared pointer to a subscriber object that subscribes to messages of type std_msgs::msg::String.
     * 
     * This subscriber receives serial messages from the serial node.
     */
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _serialMessageSubscriber;

    /**
     * @brief A shared pointer to a timer object that triggers the publishArmState function every 1000 milliseconds.
     */
    rclcpp::TimerBase::SharedPtr _timer;

    
    /**
     * @brief A map that stores joint indices as keys and joint angles as values.
     * 
     * This map is used to represent the state of the arm, where the joint indices
     * are integers and the joint angles are doubles.
     */
    std::map<int, double> joints
    = {
        {0, 0.0},
        {1, 0.0},
        {2, 0.0},
        {3, 0.0},
        {4, 0.0},
        {5, 0.0},
        {6, 0.0}
    };

    /**
     * @brief Builds the arm state message based on the received serial message.
     * 
     * This method takes a shared pointer to a std_msgs::msg::String message and builds the arm state message
     * by parsing the received serial message.
     *
     * @param message A shared pointer to a std_msgs::msg::String message.
     */
    void build_arm_state_msg(std_msgs::msg::String::SharedPtr message);

    /**
     * @brief Builds the arm's world.
     * 
     * This method builds the static transforms for the arm's world using tf2_ros::StaticTransformBroadcaster.
     */
    void buildWorld();

    /**
     * @brief Builds the joints map.
     * 
     * This method initializes the joints map with default values.
     */
    void buildJointsMap();

    /**
     * @brief Builds the arm.
     * 
     * This method builds the static transforms for the arm using tf2_ros::StaticTransformBroadcaster.
     */
    void buildArm();

    /**
     * @brief Gets the joint name based on the given servo ID.
     * 
     * This method takes a JointNames enum value and returns the corresponding joint name as a string.
     *
     * @param servoID The servo ID as a JointNames enum value.
     * @return The joint name as a string.
     */
    std::string getJointName(JointNames servoID);

    /**
     * @brief Sets the state of a joint.
     * 
     * This method takes a servo ID, position, and duration and sets the state of the corresponding joint.
     * It returns true if the joint state is successfully set, false otherwise.
     *
     * @param servoID The servo ID as an integer.
     * @param position The desired position of the joint.
     * @param duration The duration for which the joint should move to the desired position.
     * @return True if the joint state is successfully set, false otherwise.
     */
    bool setJointState(int servoID, double position, int duration);

    /**
     * @brief Converts PWM value to joint angle in radians.
     * 
     * This method takes a PulseWidth value and converts it to the corresponding joint angle in radians.
     * It also takes optional minDegree and maxDegree values to specify the range of the joint angle.
     *
     * @param PulseWidth The PWM value as an unsigned 16-bit integer.
     * @param minDegree The minimum degree of the joint angle (optional).
     * @param maxDegree The maximum degree of the joint angle (optional).
     * @return The joint angle in radians.
     */
    double convertPwmToRadian(u_int16_t PulseWidth, u_int16_t minDegree = 0, u_int16_t maxDegree = 0);

    /**
     * @brief Receives the serial message.
     * 
     * This method is the callback function for the serial message subscriber.
     * It takes a shared pointer to a std_msgs::msg::String message and receives the serial message.
     *
     * @param message A shared pointer to a std_msgs::msg::String message.
     */
    void recieveSerialMessage(const std_msgs::msg::String::SharedPtr message);

    /**
     * @brief Parses the command string.
     * 
     * This method takes a command string and parses it to extract the joint names and positions.
     * It returns a map that stores the joint names as keys and the corresponding positions as values.
     *
     * @param command The command string to parse.
     * @return A map that stores the joint names and positions.
     */
    std::map<std::string, int> parseCommand(const std::string& command);

    /**
     * @brief Handles the emergency stop command.
     *
     * This function is responsible for handling the emergency stop command. It takes a string parameter
     * representing the command and performs the necessary actions to stop the arm's movement.
     *
     * @param command The emergency stop command.
     */
    bool handleEmergencyStop(const std::string& command);

};
