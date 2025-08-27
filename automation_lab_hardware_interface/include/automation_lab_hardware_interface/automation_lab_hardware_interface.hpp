#ifndef AUTOMATION_LAB_HARDWARE_INTERFACE_HPP
#define AUTOMATION_LAB_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace automation_lab_hardware_interface
{
class AutomationLabHardware : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    size_t num_joints_;
    std::vector<double> position_commands_;
    std::vector<double> position_states_;

    // Publisher and Subscriber
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr encoder_angle_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_angle_subscriber_;

    void target_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);
    double get_encoder_angle(size_t joint_index);
    void send_command_to_motor(size_t joint_index, double position);

    rclcpp::Node::SharedPtr node_;
};
} // namespace automation_lab_hardware_interface

#endif // AUTOMATION_LAB_HARDWARE_INTERFACE_HPP

