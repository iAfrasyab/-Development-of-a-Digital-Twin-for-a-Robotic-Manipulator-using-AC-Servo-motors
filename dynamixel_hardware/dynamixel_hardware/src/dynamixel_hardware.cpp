#include "dynamixel_hardware/dynamixel_hardware.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/float64.hpp>

namespace dynamixel_hardware
{

hardware_interface::CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    num_joints_ = info.joints.size();
    position_commands_.resize(num_joints_, 0.0);
    position_states_.resize(num_joints_, 0.0);

    // ROS2 Node initialization
    node_ = rclcpp::Node::make_shared("dynamixel_hardware");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // Subscriber for encoder_angle
    encoder_angle_1_subscriber_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/encoder_angle_1", 10,
        std::bind(&DynamixelHardware::encoder_angle_1_callback, this, std::placeholders::_1));

    encoder_angle_2_subscriber_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/encoder_angle_2", 10,
        std::bind(&DynamixelHardware::encoder_angle_2_callback, this, std::placeholders::_1));

    encoder_angle_3_subscriber_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/encoder_angle_3", 10,
        std::bind(&DynamixelHardware::encoder_angle_3_callback, this, std::placeholders::_1));

    target_angle_1_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
        "/target_angle_1", 10);

    target_angle_2_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
        "/target_angle_2", 10);

    target_angle_3_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
        "/target_angle_3", 10);


    RCLCPP_INFO(node_->get_logger(), "Hardware interface initialized with ROS2 Subscriber for encoder_angle.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
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

hardware_interface::return_type DynamixelHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (size_t i = 0; i < num_joints_; i++)
    {
        position_states_[i] = position_states_[i];
        RCLCPP_DEBUG(node_->get_logger(), "Joint %zu state: %f", i, position_states_[i]);
    }
    return hardware_interface::return_type::OK;
}


hardware_interface::return_type DynamixelHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    auto msg1 = std_msgs::msg::Float64();
    auto msg2 = std_msgs::msg::Float64();
    auto msg3 = std_msgs::msg::Float64();


    // Optionally, log the commands for debugging purposes
        msg1.data = position_commands_[0];  // Use the position_commands_ vector directly
        msg2.data = position_commands_[1];  // Use the position_commands_ vector directly
        msg3.data = position_commands_[2];  // Use the position_commands_ vector directly
        target_angle_1_publisher_->publish(msg1);
        target_angle_2_publisher_->publish(msg2);
        target_angle_3_publisher_->publish(msg3);

        //RCLCPP_DEBUG(node_->get_logger(), "Joint %zu command: %f", i, position_commands_[i]);

    return hardware_interface::return_type::OK;
}

void DynamixelHardware::encoder_angle_1_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
        // Update state for the first joint based on encoder feedback

        position_states_[0] = msg->data;
        RCLCPP_INFO(node_->get_logger(), "Updated encoder angle for joint 0: %f", msg->data);

}

void DynamixelHardware::encoder_angle_2_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
        // Update state for the first joint based on encoder feedback

        position_states_[1] = msg->data;
        RCLCPP_INFO(node_->get_logger(), "Updated encoder angle for joint 0: %f", msg->data);

}

void DynamixelHardware::encoder_angle_3_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
        // Update state for the first joint based on encoder feedback

        position_states_[2] = msg->data;
        RCLCPP_INFO(node_->get_logger(), "Updated encoder angle for joint 0: %f", msg->data);

}

double DynamixelHardware::get_encoder_angle(size_t joint_index)
{
    return position_states_[joint_index];
}

void DynamixelHardware::send_command_to_motor(size_t joint_index, double position)
{
    RCLCPP_INFO(node_->get_logger(), "Sending command to joint %zu: %f", joint_index, position);
}

} // namespace dynamixel_hardware

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)

