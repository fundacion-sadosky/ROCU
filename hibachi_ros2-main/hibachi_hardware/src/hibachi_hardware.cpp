#include "hibachi_hardware/hibachi_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hibachi_base
{

    static const std::string HW_NAME = "HibachiHardware";

    hardware_interface::CallbackReturn HibachiHardware::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Name: %s", info_.name.c_str());

        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Number of Joints %zu", info_.joints.size());

        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_position_offset_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        // wheel_diameter_ = std::stod(info_.hardware_parameters["wheel_diameter"]);
        // max_accel_ = std::stod(info_.hardware_parameters["max_accel"]);
        // max_speed_ = std::stod(info_.hardware_parameters["max_speed"]);
        // polling_timeout_ = std::stod(info_.hardware_parameters["polling_timeout"]);

        // _serialPort.setPort("/dev/ttyACM0");
        serial_port_name_ = info_.hardware_parameters["serial_port"];
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Port: %s", serial_port_name_.c_str());
        _serialPort.setPort(serial_port_name_);
        _serialPort.openConnection();

        // serial_port_ = info_.hardware_parameters["serial_port"];

        // RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Port: %s", serial_port_.c_str());
        // horizon_legacy::connect(serial_port_);
        // horizon_legacy::configureLimits(max_speed_, max_accel_);
        // resetTravelOffset();

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            // HuskyHardware has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
            RCLCPP_FATAL(
                rclcpp::get_logger(HW_NAME),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
            RCLCPP_FATAL(
                rclcpp::get_logger(HW_NAME),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
            RCLCPP_FATAL(
                rclcpp::get_logger(HW_NAME),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
            RCLCPP_FATAL(
                rclcpp::get_logger(HW_NAME),
                "Joint '%s' have '%s' as first state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
            RCLCPP_FATAL(
                rclcpp::get_logger(HW_NAME),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;

    }

    hardware_interface::CallbackReturn HibachiHardware::on_configure(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Configuring");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HibachiHardware::on_cleanup(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Cleaning up");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HibachiHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Activating");

        // set some default values
        for (auto i = 0u; i < hw_states_position_.size(); i++)
        {
            if (std::isnan(hw_states_position_[i]))
            {
            hw_states_position_[i] = 0;
            hw_states_position_offset_[i] = 0;
            hw_states_velocity_[i] = 0;
            hw_commands_[i] = 0;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System Successfully started!");

        return hardware_interface::CallbackReturn::SUCCESS;

    }

                
    hardware_interface::CallbackReturn HibachiHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Deactivating");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    
    hardware_interface::CallbackReturn HibachiHardware::on_shutdown(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Shutting down");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    
    hardware_interface::CallbackReturn HibachiHardware::on_error(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Handling error (?)");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> HibachiHardware::export_state_interfaces()
    {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    }

    return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> HibachiHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // Husky implementation
        /* for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));

            // Detmerine which joints will be used for commands since Husky only has two motors
            if (info_.joints[i].name == LEFT_CMD_JOINT_NAME)
            {
            left_cmd_joint_index_ = i;
            }

            if (info_.joints[i].name == RIGHT_CMD_JOINT_NAME)
            {
            right_cmd_joint_index_ = i;
            }
        } */

        // Rosbot implementation
        /* for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_commands_[info_.joints[i].name]));
        } */

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
        }

        return command_interfaces;
    }


    hardware_interface::return_type HibachiHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Reading from hardware");

        // RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Reading from hardware");
        updateJointsFromHardware();

        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully read!");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HibachiHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Writing to hardware");

        // RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Writing to hardware");
        writeCommandsToHardware();

        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully written!");

        return hardware_interface::return_type::OK;
    }

    void HibachiHardware::updateJointsFromHardware()
    {
        hibachi_base::GetFourWheelEncoder getFourWheelEncoder;
        _serialPort.sendMessage(&getFourWheelEncoder);
    	// auto fourWheelEncoderData = (FourWheelEncoderData*)_serialPort.waitMessage(FourWheelEncoderData::MESSAGE_TYPE, 1.0);
        auto fourWheelEncoderPosition = (FourWheelEncoderPosition*)_serialPort.waitMessage(FourWheelEncoderPosition::MESSAGE_TYPE, 1.0);
      	auto fourWheelEncoderSpeed = (FourWheelEncoderSpeed*)_serialPort.waitMessage(FourWheelEncoderSpeed::MESSAGE_TYPE, 1.0);

        double temp_fl, temp_fr, temp_rl, temp_rr;
        if (fourWheelEncoderPosition != NULL)
	    {
            fourWheelEncoderPosition->getWheelsAngPosition(temp_fl, temp_fr, temp_rl, temp_rr);
            // Acá tengo que asignar los valores a las juntas
            hw_states_position_[0] = temp_fl;
            hw_states_position_[1] = temp_fr;
            hw_states_position_[2] = temp_rl;
            hw_states_position_[3] = temp_rr;
            RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Recived pos: %f, %f, %f, %f",
                 temp_fl, temp_fr, temp_rl, temp_rr);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "No valid encoder position received");
        }

        if (fourWheelEncoderSpeed != NULL)
        {
        fourWheelEncoderSpeed->getWheelsAngSpeed(temp_fl, temp_fr, temp_rl, temp_rr);
            hw_states_velocity_[0] = temp_fl;
            hw_states_velocity_[1] = temp_fr;
            hw_states_velocity_[2] = temp_rl;
            hw_states_velocity_[3] = temp_rr;
            RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Recived speed: %f, %f, %f, %f",
                 temp_fl, temp_fr, temp_rl, temp_rr);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "No valid encoder speed received");
        }

    }

    void HibachiHardware::writeCommandsToHardware()
    {
        double diff_speed_front_left =  hw_commands_[0];
        double diff_speed_front_right = hw_commands_[1];
        double diff_speed_rear_left =   hw_commands_[2];
        double diff_speed_rear_right =  hw_commands_[3];
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "I am sending %f, %f, %f and %f", diff_speed_front_left, diff_speed_front_right, diff_speed_rear_left, diff_speed_rear_right);

        hibachi_base::SetSkidSteerMotorSpeed SetSkidSteerMotorSpeed(diff_speed_front_left, diff_speed_front_right, diff_speed_rear_left, diff_speed_rear_right);
        _serialPort.sendMessage(&SetSkidSteerMotorSpeed);
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hibachi_base::HibachiHardware, hardware_interface::SystemInterface)