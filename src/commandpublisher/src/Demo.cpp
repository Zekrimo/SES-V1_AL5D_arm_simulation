#include "../include/Demo.hpp"

Demo::Demo(std::string customArg) : Node("command_publisher")
{
    _serialMessagePublisher = this->create_publisher<std_msgs::msg::String>("serial_messages", 10);
    // Create the timer, but don't start it yet
    _timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Demo::processSerialMessage, this));

    std::cout << customArg << std::endl;

    if (customArg == "false")
    {
        _customMessageProvided = false;
        //RCLCPP_INFO(this->get_logger(), "No custom string provided, using default messages.");
    }
    else
    {
        _customMessage = customArg; // Store the argument for use
        //RCLCPP_INFO(this->get_logger(), "Custom string provided: %s", _customMessage.c_str());
        _customMessageProvided = true;
    }

    // Initialize the default pose
    initDefaultPose();
}

Demo::~Demo()
{
}

void Demo::processSerialMessage()
{
    if (_customMessageProvided)
    {
        sendSerialMessage(_customMessage);
        if (_debug)
        {
            std::cout << "Sent message: '" << _customMessage << "' send" << std::endl;
        }
        rclcpp::shutdown();
    }
    else
    {
        sendTestMessage();
        rclcpp::shutdown();
    }
}

void Demo::sendTestMessage()
{
    // Send the test message
    sendSerialMessage(MESSAGE1);
    if (_debug)
    {
        std::cout << "Sent message: '" << MESSAGE1 << "' send" << std::endl;
    }

    sendSerialMessage(MESSAGE2);
    if (_debug)
    {
        std::cout << "Sent message: '" << MESSAGE2 << "' send" << std::endl;
    }

    sendSerialMessage(MESSAGE3);
    if (_debug)
    {
        std::cout << "Sent message: '" << MESSAGE3 << "' send" << std::endl;
    }

    sendSerialMessage(MESSAGE4);
    if (_debug)
    {
        std::cout << "Sent message: '" << MESSAGE4 << "' send" << std::endl;
    }

    sleep(10);

    sendSerialMessage(CLOSEGRIPPERL);
    if (_debug)
    {
        std::cout << "Sent message: '" << CLOSEGRIPPERL << "' send" << std::endl;
    }

    sendSerialMessage(CLOSEGRIPPERR);
    if (_debug)
    {
        std::cout << "Sent message: '" << CLOSEGRIPPERR << "' send" << std::endl;
    }

    sleep(10);

    sendSerialMessage(DEFAULTS1POS);
    if (_debug)
    {
        std::cout << "Sent message: '" << DEFAULTS1POS << "' send" << std::endl;
    }

    sendSerialMessage(DEFAULTS2POS);
    if (_debug)
    {
        std::cout << "Sent message: '" << DEFAULTS2POS << "' send" << std::endl;
    }

    sendSerialMessage(DEFAULTS3POS);
    if (_debug)
    {
        std::cout << "Sent message: '" << DEFAULTS3POS << "' send" << std::endl;
    }

    sleep(10);

    sendSerialMessage(OPENGRIPPERL);
    if (_debug)
    {
        std::cout << "Sent message: '" << OPENGRIPPERL << "' send" << std::endl;
    }

    sendSerialMessage(OPENGRIPPERR);
    if (_debug)
    {
        std::cout << "Sent message: '" << OPENGRIPPERR << "' send" << std::endl;
    }

    // Stop the timer
    _timer.reset();
}

void Demo::sendSerialMessage(std::string message)
{
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = message;
    _serialMessagePublisher->publish(*msg);
}

void Demo::initDefaultPose()
{
    sendSerialMessage(DEFAULTS0POS);
    sendSerialMessage(DEFAULTS1POS);
    sendSerialMessage(DEFAULTS2POS);
    sendSerialMessage(DEFAULTS3POS);
    sendSerialMessage(DEFAULTS4POS);
    sendSerialMessage(DEFAULTS5POS);

    sleep(5);
}
