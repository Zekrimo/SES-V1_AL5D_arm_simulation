#include "../include/ArmStatePublisher.h"
#include "ArmStatePublisher.h"

ArmStatePublisher::ArmStatePublisher() : Node("arm_state_publisher"), _tf2StaticBroadcaster_(this)
{
        buildWorld();

        this->_armStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        this->_serialMessageSubscriber = this->create_subscription<std_msgs::msg::String>("serial_messages", 10, std::bind(&ArmStatePublisher::recieveSerialMessage, this, std::placeholders::_1));

        this->_timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ArmStatePublisher::publishArmState, this));

        armStateMsg.name = {
            "base_link2turret",
            "turret2upperarm",
            "upperarm2forearm",
            "forearm2wrist",
            "wrist2hand",
            "gripper_left2hand",
            "gripper_right2hand"};
        armStateMsg.position = {
            0, // base
            0, // shoulder
            0, // elbow
            0, // wrist
            0, // hand
            0, // gripper 01
            0, // gripper 02
        };
}

ArmStatePublisher::~ArmStatePublisher()
{
}

void ArmStatePublisher::publishArmState()
{
        if (this->joints.empty())
        {
                return;
        }

        // set timestamp
        armStateMsg.header.stamp = this->now();

        // set position
        // RCLCPP_INFO(this->get_logger(), "send commmand to simulation:");
        for (auto const &[jointName, jointPos] : this->joints)
        {

                armStateMsg.position[jointName] = jointPos;

                if (debug)
                {
                        RCLCPP_INFO(this->get_logger(), "     joint name: %s", armStateMsg.name[jointName].c_str());
                        RCLCPP_INFO(this->get_logger(), "     joint position: %f", armStateMsg.position[jointName]);
                }
        }

        // send over publisher
        this->_armStatePublisher->publish(armStateMsg);
}

void ArmStatePublisher::buildWorld()
{
        buildArm();
}


void ArmStatePublisher::buildArm()
{
        // Create a arm frame
        geometry_msgs::msg::TransformStamped armTransform;

        // Create a arm frame
        armTransform.header.frame_id = "map";

        // Set the name of the arm frame
        armTransform.child_frame_id = "base_link"; // needs to refer to link of arm URDF

        // Set the position, velocity, and effort
        armTransform.transform.translation.x = 0.0;
        armTransform.transform.translation.y = 0.0;
        armTransform.transform.translation.z = 0.0;

        // Set the timestamp
        armTransform.header.stamp = this->now();

        // broadcast the arm frame
        _tf2StaticBroadcaster_.sendTransform(armTransform);
}

std::string ArmStatePublisher::getJointName(JointNames servoID)
{
        switch (servoID)
        {
        case JointNames::BASE_LINK2TURRET:
                return "base_link2turret";
        case JointNames::TURRET2UPPERARM:
                return "turrt2upperarm";
        case JointNames::UPPERARM2FOREARM:
                return "upperarm2forearm";
        case JointNames::FOREARM2WRIST:
                return "forearm2wrist";
        case JointNames::WRIST2HAND:
                return "wrist2hand";
        case JointNames::GRIPPER_LEFT2HAND:
                return "gripper_left2hand";
        case JointNames::GRIPPER_RIGHT2HAND:
                return "grpper_right2hand";
        default:
                return "";
        }
        return "";
}

bool ArmStatePublisher::setJointState(int servoID, double position, int duration)
{
        // Create a lambda function that updates the joint position by `Increment` every millisecond
        // labda inside of functon body because its not used anywhere else
        auto updateJoint = [this](double Increment, int servoID, double endPosition, int time)
        {
                if(!emergencyStop)
                {

                for (int i = 0; i < time; i++)
                {
                        // chcek if servoID is valid
                        if (joints.find(servoID) != joints.end())
                        {
                                joints[servoID] += Increment;
                                if (debug)
                                {
                                       // RCLCPP_INFO(this->get_logger(), "current position: %f", this->joints.at(servoID));
                                       // RCLCPP_INFO(this->get_logger(), "servo incremented with %f", Increment);
                                }
                        }
                        else
                        {
                                // Handle the case where the provided servoID is invalid
                                RCLCPP_WARN(this->get_logger(), "Invalid servoID: %d", servoID);
                                return false;
                        }
                        if (debug)
                        {
                               // RCLCPP_INFO(this->get_logger(), "servoID: %d, new update position: %f", servoID, this->joints.at(servoID));
                        }

                        // Create a timer with the specified duration and callback function
                        _timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ArmStatePublisher::publishArmState, this));
                }
                this->joints.at(servoID) = endPosition;
                return true;
                }
        return false;
        };

        ////setJointState fucntion body
        //prints for debug
        if (debug)
        {
                RCLCPP_INFO(this->get_logger(), "setJointState");
                RCLCPP_INFO(this->get_logger(), "duration: %d", duration);
        }

        //check if duration is valid
        if (duration == 0 || duration > maxDuration)
        {
                // make sure to not devide by 0
                if (duration == 0)
                {
                        RCLCPP_WARN(this->get_logger(), "WARNING: known bug! recieved duration is 0 even though its parsed correcly");
                }
                //warn user that specified duration has changed to new value
                RCLCPP_WARN(this->get_logger(), "duration is invalid, setting to 5000");
                duration = defaultTimeMs;
        }

        // calculate differnce between current position and new position
        double difference = position - this->joints.at(servoID);

        // calculate increment over specified duration
        double increment = difference / duration;
        if (debug)
        {
                RCLCPP_INFO(this->get_logger(), "difference: %f", difference);
                RCLCPP_INFO(this->get_logger(), "increment: %f", increment);
        }

        // Create a thread that updates the call the lambda function above
        std::thread thread(updateJoint, increment, servoID, position, duration);

        if (debug)
        {
                RCLCPP_INFO(this->get_logger(), "send");
        }

        //detach thread so it is no longer my problem to handle :)
        thread.detach();

        return true;
}

double ArmStatePublisher::convertPwmToRadian(u_int16_t PulseWidth, u_int16_t minDegree, u_int16_t maxDegree)
{
        // Ensure PWM value is within the expected range
        if (PulseWidth < 500)
        {
                PulseWidth = 500;
        }
        else if (PulseWidth > 2500)
        {
                PulseWidth = 2500;
        }

        if (PulseWidth < minDegree && minDegree != 0)
        {
                PulseWidth = minDegree;
        }
        else if (PulseWidth > maxDegree && maxDegree != 0)
        {
                PulseWidth = maxDegree;
        }

        // Convert PWM value to angle in degrees. Assuming linear mapping:
        double degrees = (PulseWidth - 500) * (180.0 / (2500 - 500)) - 90.0;

        // Convert degrees to radians
        double radians = degrees * (M_PI / 180.0);

        if (debug)
        {
                RCLCPP_INFO(this->get_logger(), "PulseWidth: %d, degrees: %f, radians: %f", PulseWidth, degrees, radians);
        }

        return radians;
}

void ArmStatePublisher::recieveSerialMessage(const std_msgs::msg::String::SharedPtr message)
{
        //make sure to handle emergency stop
        if(handleEmergencyStop(message->data))
        {
                return;
        }

        //conver to command map
        std::map<std::string, int> commandMap = parseCommand(message->data);

        if (debug)
        {
                std::string commandDebug = "recieved: " + message->data;
                RCLCPP_INFO(this->get_logger(), commandDebug.c_str());
                RCLCPP_INFO(this->get_logger(), "COMMAND MAP: servoID: %d, pulseWidth: %d, time: %d", commandMap["servoID"], commandMap["pulseWidth"], commandMap["time"]);
        }



        //set the new joint state
        setJointState(commandMap["servoID"],
                      convertPwmToRadian(commandMap["pulseWidth"]),
                      commandMap["time"]);

        return;
}

std::map<std::string, int> ArmStatePublisher::parseCommand(const std::string &command)
{
        std::map<std::string, int> commandMap;
        size_t posP = command.find('P');
        size_t posT = command.find('T');

        if (posP == std::string::npos || posT == std::string::npos || posP >= posT)
        {
                throw std::invalid_argument("Invalid command format.");
        }

        try
        {
                commandMap["servoID"] = std::stoi(command.substr(1, posP - 1));
                commandMap["pulseWidth"] = std::stoi(command.substr(posP + 1, posT - posP - 1));
                commandMap["time"] = std::stoi(command.substr(posT + 1, command.length() - posT - 1));

                if (debug)
                {
                        RCLCPP_INFO(this->get_logger(), "recieved: %s", command.c_str());
                        RCLCPP_INFO(this->get_logger(), "Paresed with servoID: %d, pulseWidth: %d, time: %d", commandMap["servoID"], commandMap["pulseWidth"], commandMap["time"]);
                }
        }
        catch (const std::invalid_argument &e)
        {
                throw std::invalid_argument("Error parsing command: " + std::string(e.what()));
        }
        catch (const std::out_of_range &e)
        {
                throw std::out_of_range("Error parsing command: " + std::string(e.what()));
        }

        return commandMap;
}

bool ArmStatePublisher::handleEmergencyStop(const std::string &command)
{
        //RCLCPP_INFO(this->get_logger(), "handleEmergencyStop");
        //RCLCPP_INFO(this->get_logger(), "command: %s", command.c_str());

        if (command == "STOP")
        {
                RCLCPP_WARN(this->get_logger(), "Emergency stop activated, call START to deactivate");
                emergencyStop = true;
                return true;
        }
        else if (command == "START")
        {
                RCLCPP_WARN(this->get_logger(), "Emergency stop deactivated");
                emergencyStop = false;
                return true;
        }
        else 
        {
                return false;
        }
}
