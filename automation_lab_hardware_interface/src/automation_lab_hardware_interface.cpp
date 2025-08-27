#include "automation_lab_hardware_interface/automation_lab_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/float64.hpp>

namespace automation_lab_hardware_interface
{

hardware_interface::CallbackReturn AutomationLabHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    num_joints_ = info.joints.size();
    position_commands_.resize(num_joints_, 0.0);
    position_states_.resize(num_joints_, 0.0);

    // ROS2 Node initialization
    node_ = rclcpp::Node::make_shared("automation_lab_hardware_interface");

    // Publisher
    encoder_angle_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/encoder_angle", 10);

    // Subscriber
    target_angle_subscriber_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/target_angle", 10,
        std::bind(&AutomationLabHardware::target_angle_callback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "Hardware interface initialized with ROS2 Publisher and Subscriber.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AutomationLabHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < num_joints_; i++)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, "position", &position_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AutomationLabHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < num_joints_; i++)
    {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, "position", &position_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::return_type AutomationLabHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (size_t i = 0; i < num_joints_; i++)
    {
        position_states_[i] = get_encoder_angle(i);

        // Publish encoder angle
        std_msgs::msg::Float64 msg;
        msg.data = position_states_[i];
        encoder_angle_publisher_->publish(msg);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AutomationLabHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (size_t i = 0; i < num_joints_; i++)
    {
        send_command_to_motor(i, position_commands_[i]);
    }
    return hardware_interface::return_type::OK;
}

void AutomationLabHardware::target_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Assuming the first joint for simplicity
    if (num_joints_ > 0)
    {
        position_commands_[0] = msg->data;
        RCLCPP_INFO(node_->get_logger(), "Received target angle: %f", msg->data);
    }
}

double AutomationLabHardware::get_encoder_angle(size_t joint_index)
{
    return position_states_[joint_index];
}

void AutomationLabHardware::send_command_to_motor(size_t joint_index, double position)
{
    RCLCPP_INFO(node_->get_logger(), "Sending command to joint %zu: %f", joint_index, position);
}

} // namespace automation_lab

PLUGINLIB_EXPORT_CLASS(automation_lab_hardware_interface::AutomationLabHardware, hardware_interface::SystemInterface)

