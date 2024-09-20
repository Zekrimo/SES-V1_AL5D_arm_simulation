#include "CupStateSubscriber.h"


// Constructor
CupStateSubscriber::CupStateSubscriber() 
    : Node("cup_state_subscriber")
{

    _tf2Buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(30.0));

    _tf2Listener = std::make_shared<tf2_ros::TransformListener>(*_tf2Buffer, this, false);

    _cupStatePublisher = this->create_publisher<geometry_msgs::msg::TransformStamped>("cup_update", 10);

    _timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CupStateSubscriber::timerTriggered, this));


}

// Destructor
CupStateSubscriber::~CupStateSubscriber() {
    // Clean up any resources here
}

void CupStateSubscriber::timerTriggered()
{
     geometry_msgs::msg::TransformStamped msg;
        try
        {
            // Attempt to get the latest transform for each relevant pair
            auto transformGripperLeft = _tf2Buffer->lookupTransform("gripper_left", "cup_base_link", tf2::TimePointZero);
            auto transformGripperRight = _tf2Buffer->lookupTransform("gripper_right", "cup_base_link", tf2::TimePointZero);
            auto transformHand = _tf2Buffer->lookupTransform("hand", "cup_base_link", tf2::TimePointZero);

            // Check if the cup is within the grip thresholds for both grippers
            if (isCupInGrip(transformGripperLeft) && isCupInGrip(transformGripperRight))
            {
                msg = transformHand;
                //RCLCPP_INFO(this->get_logger(), "sending arm transform as cup transform");
            }
            else
            {
                msg = _tf2Buffer->lookupTransform("base_link", "cup_base_link", tf2::TimePointZero);
                // RCLCPP_INFO(this->get_logger(), "CupStateSubscriber::timerTriggered() called");

            }

            _cupStatePublisher->publish(msg);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
            RCLCPP_WARN(this->get_logger(), "CupStateSubscriber::timerTriggered() failed to publish message");
        }

}

bool CupStateSubscriber::isCupInGrip(const geometry_msgs::msg::TransformStamped &transform)
{
    // RCLCPP_INFO(this->get_logger(), " transform.translation.x: %f", transform.transform.translation.x, "margin: %f", _positivePickupMarginX);
    // RCLCPP_INFO(this->get_logger(), " transform.translation.y: %f", transform.transform.translation.y, "margin: %f", _positivePickupMarginY);
    // RCLCPP_INFO(this->get_logger(), " transform.translation.z: %f", transform.transform.translation.z, "margin: %f", _positivePickupMarginZ);

    // if(transform.transform.translation.x > _positivePickupMarginX || transform.transform.translation.x < _negativePickupMarginX)
    // {
    //     RCLCPP_INFO(this->get_logger(), "X not in margin");
    //     RCLCPP_INFO(this->get_logger(), "x is %f", transform.transform.translation.x, "margin is between %f and %f", _negativePickupMarginX, _positivePickupMarginX);
    // }

    // if(transform.transform.translation.y > _positivePickupMarginY || transform.transform.translation.y < _negativePickupMarginY)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Y not in margin");
    //     RCLCPP_INFO(this->get_logger(), "y is %f", transform.transform.translation.y, "margin is between %f and %f", _negativePickupMarginY, _positivePickupMarginY);
    // }

    // if(transform.transform.translation.z > _positivePickupMarginZ || transform.transform.translation.z < _negativePickupMarginZ)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Z not in margin");
    //     RCLCPP_INFO(this->get_logger(), "z is %f", transform.transform.translation.z, "margin is between %f and %f", _negativePickupMarginZ, _positivePickupMarginZ);
    // }




    // Check if the cup is within the grip thresholds for both grippers
    if (transform.transform.translation.x < _positivePickupMarginX && transform.transform.translation.x > _negativePickupMarginX &&
        transform.transform.translation.y < _positivePickupMarginY && transform.transform.translation.y > _negativePickupMarginY &&
        transform.transform.translation.z < _positivePickupMarginZ && transform.transform.translation.z > _negativePickupMarginZ)
    {
        if(_debug) {RCLCPP_INFO(this->get_logger(), "Cup is in grip margin");}
        return true;
    }
    else
    {
        if(_debug) {RCLCPP_INFO(this->get_logger(), "Cup is NOT in grip margin");}
        return false;
    }
}
