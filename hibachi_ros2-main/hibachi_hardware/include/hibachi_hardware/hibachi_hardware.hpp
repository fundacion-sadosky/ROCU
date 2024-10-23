#ifndef HIBACHI_BASE__HIBACHI_HARDWARE_HPP_
#define HIBACHI_BASE__HIBACHI_HARDWARE_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
// #include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hibachi_hardware/serial_port.h"

#include "rclcpp/rclcpp.hpp"

namespace hibachi_base
{
    class HibachiHardware : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(HibachiHardware)

            HARDWARE_INTERFACE_PUBLIC
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;

            HARDWARE_INTERFACE_PUBLIC
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

            HARDWARE_INTERFACE_PUBLIC
            hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

            HARDWARE_INTERFACE_PUBLIC
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

            HARDWARE_INTERFACE_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

            HARDWARE_INTERFACE_PUBLIC
            hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

            HARDWARE_INTERFACE_PUBLIC
            hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

            HARDWARE_INTERFACE_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            HARDWARE_INTERFACE_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            HARDWARE_INTERFACE_PUBLIC
            hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

            HARDWARE_INTERFACE_PUBLIC
            hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        private:
            void writeCommandsToHardware();
            void updateJointsFromHardware();

            std::vector<double> hw_commands_;
            std::vector<double> hw_states_position_, hw_states_position_offset_, hw_states_velocity_;

            std::string serial_port_name_;
            serial::Serial _serial;
            hibachi_base::SerialPort _serialPort;
    };
    
} // namespace hibachi_base

#endif  // HIBACHI_BASE__HIBACHI_HARDWARE_HPP_