#ifndef DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP
#define DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP

#include "hardware_interface/system_interface.hpp"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace dynamixel_hardware
{
class DynamixelHardware : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    size_t num_joints_;  // Number of joints
    std::vector<double> position_commands_; // Target angles (commands)
    std::vector<double> position_states_;   // Encoder angles (states)

    // ROS 2 Publisher and Subscriber
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_angle_1_publisher_; // Publishes target_angle
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_angle_2_publisher_; // Publishes target_angle
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_angle_3_publisher_; // Publishes target_angle
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr encoder_angle_1_subscriber_; // Subscribes to encoder_angle
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr encoder_angle_2_subscriber_; // Subscribes to encoder_angle
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr encoder_angle_3_subscriber_; // Subscribes to encoder_angle
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;

    // ROS 2 Node
    rclcpp::Node::SharedPtr node_;

    // Callback for receiving encoder_angle
    void encoder_angle_1_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void encoder_angle_2_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void encoder_angle_3_callback(const std_msgs::msg::Float64::SharedPtr msg);

    // Get encoder angle for a joint
    double get_encoder_angle(size_t joint_index);

    // Send command to motor
    void send_command_to_motor(size_t joint_index, double position);
};
} // namespace dynamixel_hardware

#endif // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP

