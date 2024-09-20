#include "CupStatePublisher.h"

CupStatePublisher::CupStatePublisher(/* args */) : Node("cup_state_publisher")
{
    _state = State::IDLE;

    _fallSpeed = 0.005;

    _groundLevel = -0.05;

    _startXpos =  -0.1;
    _startYpos = -0.33;

    _tf2Broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    _cupSubscriber = this->create_subscription<geometry_msgs::msg::TransformStamped>("cup_update", 10, std::bind(&CupStatePublisher::handleCup, this, std::placeholders::_1));

    _timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CupStatePublisher::publishCupState, this));

    std::string cupURDF = this->declare_parameter("cup_description", std::string(""));

    _cupDescriptionPublisher = this->create_publisher<std_msgs::msg::String>("cup_description", rclcpp::QoS(1).transient_local());

    if (!cupURDF.empty())
    {
        auto descriptionMsg = std::make_unique<std_msgs::msg::String>();
        descriptionMsg->data = cupURDF;
        _cupDescriptionPublisher->publish(std::move(descriptionMsg));
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "No cup description provided");
    }

    initCupPoseMsg();
}

CupStatePublisher::~CupStatePublisher()
{
}

void CupStatePublisher::publishCupState()
{
    _cupPoseMsg.header.stamp = this->now();

    // make sure it never goes throught the floor.
    if (_cupPoseMsg.transform.translation.z < _groundLevel)
    {
        (_cupPoseMsg.transform.translation.z = _groundLevel);
    }

    _tf2Broadcaster->sendTransform(_cupPoseMsg);
}

void CupStatePublisher::initCupPoseMsg()
{
    _cupPoseMsg.header.frame_id = "base_link";
    _cupPoseMsg.child_frame_id = "cup_base_link";

    // place cup in front of the robot values correspont to preprogrammed path in commandpublisher package
    _cupPoseMsg.transform.translation.x = _startXpos;
    _cupPoseMsg.transform.translation.y = _startYpos;
    _cupPoseMsg.transform.translation.z = _groundLevel;

    // make sure crop stays upright
    _cupPoseMsg.transform.rotation.x = 0.0;
    _cupPoseMsg.transform.rotation.y = 0.0;
    _cupPoseMsg.transform.rotation.z = 0.0;
    _cupPoseMsg.transform.rotation.w = 1.0;
}

void CupStatePublisher::handleCup(const geometry_msgs::msg::TransformStamped &msg)
{
    // RCLCPP_INFO(this->get_logger(), "CupStatePublisher::handleCup() called");
    if (msg.header.frame_id == "hand")
    {
        RCLCPP_INFO(this->get_logger(), "Cup is in hand");
        _cupPoseMsg = msg;
        setCupPoseMsg(_cupPoseMsg);
        _state = State::GRABBED;
    }
    else if (msg.header.frame_id == "base_link")
    {
        if (msg.transform.translation.z > _groundLevel)
        {
            _cupPoseMsg = msg;

            // RCLCPP_INFO(this->get_logger(), "Cup is falling");
            affectCupByGravitationalForce();
            _state = State::FALLING;
        }
        else
        {
            // RCLCPP_INFO(this->get_logger(), "Cup has landed");
            _state = State::IDLE;
        }
    }
    else
    {
        _state = State::IDLE;
        // RCLCPP_INFO(this->get_logger(), "Cup is not idle");
    }
}

void CupStatePublisher::setCupPoseMsg(const geometry_msgs::msg::TransformStamped &msg)
{
    _cupPoseMsg = msg;
}

void CupStatePublisher::affectCupByGravitationalForce()
{
    RCLCPP_INFO(this->get_logger(), "Cup is falling");

    // make sure crop stays upright while falling
    _cupPoseMsg.transform.rotation.y = 0.0;
    _cupPoseMsg.transform.rotation.z = 0.0;
    _cupPoseMsg.transform.rotation.x = 0.0;
    _cupPoseMsg.transform.rotation.w = 1.0;

    if (_cupPoseMsg.transform.translation.z <= _groundLevel)
    {
        RCLCPP_INFO(this->get_logger(), "Cup below ground level");
        _cupPoseMsg.transform.translation.z = _groundLevel;
        return;
    }
    else
    {
        // drop cup at fallspeed
        _cupPoseMsg.transform.translation.z -= _fallSpeed;
    }
}
