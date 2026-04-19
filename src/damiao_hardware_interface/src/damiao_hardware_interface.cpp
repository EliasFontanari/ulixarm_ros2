#include "damiao_hardware_interface/damiao_hardware_interface.hpp"
#include <string>
#include <vector>


namespace damiao_hardware_interface
{
    CallbackReturn RobotSystem::on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params)
    {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // Load robot_description
        {
            auto node = rclcpp::Node::make_shared("hf_robot_descritption_listener");
    
            node->declare_parameter("robot_description", std::string(""));
            std::string urdf_string = node->get_parameter("robot_description").get_value<std::string>();
    
            if (urdf_string.empty()) {
                RCLCPP_ERROR(get_logger(), "robot_description is empty!");
                return CallbackReturn::ERROR;
            }
            
            // build URDF model
            try {
                pinocchio::urdf::buildModelFromXML(urdf_string, this->pin_model_);
                this->pin_data_ = pinocchio::Data(this->pin_model_);
            } catch (const std::exception & e) {
                RCLCPP_ERROR(get_logger(), "Failed to build model: %s", e.what());
                return CallbackReturn::ERROR;
            }
            
            // If we reach here, it succeeded
            RCLCPP_INFO(get_logger(), "Pinocchio model build.");
        }

        // Load motor params Kp and Kd
        {
            this->motor_kp_.resize(info_.joints.size());
            this->motor_kd_.resize(info_.joints.size());
    
            for (size_t i = 0; i < info_.joints.size(); i++)
            {
                const auto & joint = info_.joints[i];
    
                // Read Kp
                if (joint.parameters.find("Kp") != joint.parameters.end())
                    this->motor_kp_[i] = std::stod(joint.parameters.at("Kp"));
                else {
                    RCLCPP_ERROR(get_logger(), "Joint '%s' missing Kp!", joint.name.data());
                    return CallbackReturn::ERROR;
                }
    
                // Read Kd
                if (joint.parameters.find("Kd") != joint.parameters.end())
                    this->motor_kd_[i] = std::stod(joint.parameters.at("Kd"));
                else {
                    RCLCPP_ERROR(get_logger(), "Joint '%s' missing Kd!", joint.name.data());
                    return CallbackReturn::ERROR;
                }
                
                RCLCPP_INFO(get_logger(), "Joint '%s' — Kp: %.2f  Kd: %.2f",
                    joint.name.data(), motor_kp_[i], motor_kd_[i]);
                    
            }
        }

        // Load gravity compensation flag
        {
            if (info_.hardware_parameters.find("use_gravity_compensation") != 
                info_.hardware_parameters.end())
            {
                use_gravity_compensation_ = 
                    info_.hardware_parameters.at("use_gravity_compensation") == "true";
            } else {
                use_gravity_compensation_ = true; // default
            }
            
            RCLCPP_INFO(get_logger(), "Gravity compensation: %s", 
                use_gravity_compensation_ ? "ENABLED" : "DISABLED");
        }

        // Load serial params
        {
            if (info_.hardware_parameters.find("serial_port") == info_.hardware_parameters.end()) {
                RCLCPP_ERROR(get_logger(), "Missing hardware parameter 'serial_port'!");
                return CallbackReturn::ERROR;
            }
            this->port_ = info_.hardware_parameters.at("serial_port");

            if (info_.hardware_parameters.find("serial_baudrate") == info_.hardware_parameters.end()) {
                RCLCPP_ERROR(get_logger(), "Missing hardware parameter 'serial_baudrate'!");
                return CallbackReturn::ERROR;
            }
            this->baudrate_ = std::stoi(info_.hardware_parameters.at("serial_baudrate"));
        }

        // init joint names
        {
            // manipulator
            this->manipulator_joint_names_.push_back(info_.joints[0].name);     // joint1
            this->manipulator_joint_names_.push_back(info_.joints[1].name);     // joint2
            this->manipulator_joint_names_.push_back(info_.joints[2].name);     // joint3
            this->manipulator_joint_names_.push_back(info_.joints[3].name);     // joint4
            this->manipulator_joint_names_.push_back(info_.joints[4].name);     // joint5
            this->manipulator_joint_names_.push_back(info_.joints[5].name);     // joint6

            // gripper
            this->gripper_joint_names_.push_back(info_.joints[6].name);         // finger_right_joint
        }
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // reset values always when configuring hardware
        for (const auto & [name, descr] : joint_state_interfaces_)
        {
            set_state(name, 0.0);
        }
        for (const auto & [name, descr] : joint_command_interfaces_)
        {
            set_command(name, 0.0);
        }
        for (const auto & [name, descr] : sensor_state_interfaces_)
        {
            set_state(name, 0.0);
        }

        // Establish connection motor controller with serial
        {
            // Serial connection
            try {
                this->mc.emplace(this->port_.data(), static_cast<speed_t>(this->baudrate_));
            } catch (const std::exception & e) {
                RCLCPP_ERROR(get_logger(), "Failed to open serial port '%s': %s", this->port_.data(), e.what());
                return CallbackReturn::ERROR;
            }
            RCLCPP_INFO(get_logger(), "Initialized port: %s - baudrate: %d", this->port_.data(), this->baudrate_);
            
            // Initialize motors
            this->mc->add_motor(this->manipulator_joint_names_[0], damiao::DM4340,  0x01, 0x11);
            this->mc->add_motor(this->manipulator_joint_names_[1], damiao::DM4340,  0x02, 0x12);
            this->mc->add_motor(this->manipulator_joint_names_[2], damiao::DM4340,  0x03, 0x13);
            this->mc->add_motor(this->manipulator_joint_names_[3], damiao::DM4310,  0x04, 0x14);
            this->mc->add_motor(this->manipulator_joint_names_[4], damiao::DM4310,  0x05, 0x15);
            this->mc->add_motor(this->manipulator_joint_names_[5], damiao::DM4310,  0x06, 0x16);
            this->mc->add_motor(this->gripper_joint_names_[0],     damiao::DMJ3507, 0x07, 0x17);
            RCLCPP_INFO(get_logger(), "Added all motors to MotorController");
            
            // Enable motors
            RCLCPP_INFO(get_logger(), "Enabling motors...");
            try {
                this->mc->enable_motor_all();
            } catch (const std::exception & e) {
                RCLCPP_ERROR(get_logger(), "Failed to enable motors: %s", e.what());
                return CallbackReturn::ERROR;
            }
            RCLCPP_INFO(get_logger(), "Enabled all motors!");

            // Initial status refresh
            RCLCPP_INFO(get_logger(), "Refreshing motor status...");
            try {
                this->mc->refresh_motor_status_all();
            } catch (const std::exception & e) {
                RCLCPP_ERROR(get_logger(), "Failed to refresh motor status: %s", e.what());
                return CallbackReturn::ERROR;
            }
            RCLCPP_INFO(get_logger(), "Refreshed all motor status!");
        }

        // ATTENTION! overwrite the positions with ones from hardware sensors
        {
            // Manipulator
            for (const auto& joint_name : this->manipulator_joint_names_)
            {
                const auto name_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
                const double pos = this->mc->get_position(joint_name);
                set_state(name_pos, pos);
                set_command(name_pos, pos);
            }

            // Gripper
            for (const auto& joint_name : this->gripper_joint_names_)
            {
                const auto name_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
                const double pos = this->mc->get_position(joint_name) * this->gear_pinion_rot_to_lin;
                set_state(name_pos, pos);
                set_command(name_pos, pos);
            }
        }

        return CallbackReturn::SUCCESS;
    }

    return_type RobotSystem::read_manipulator()
    {
        // read values from stored data, does not polls from actual hardware
        // polling to actual hardware happens after write, as motors respond after sending command
        for (const auto& joint_name : this->manipulator_joint_names_)
        {
            const auto name_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
            const double pos = this->mc->get_position(joint_name);
            set_state(name_pos, pos);
            
            const auto name_vel = joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
            const double vel = this->mc->get_velocity(joint_name);
            set_state(name_vel, vel);
        }

        return return_type::OK;
    }

    return_type RobotSystem::read_gripper()
    {
        // const auto name_pos = info_.joints[6].name + "/" + hardware_interface::HW_IF_POSITION;
        // const double pos = this->mc->get_position(info_.joints[6].name) * this->gear_pinion_rot_to_lin;
        // set_state(name_pos, pos);
        
        // const auto name_vel = info_.joints[6].name + "/" + hardware_interface::HW_IF_VELOCITY;
        // const double vel = this->mc->get_velocity(info_.joints[6].name) * this->gear_pinion_rot_to_lin;
        // set_state(name_vel, vel);
        
        // const auto name_eff = info_.joints[6].name + "/" + hardware_interface::HW_IF_EFFORT;
        // const double effort_rot = this->mc->get_torque(info_.joints[6].name);
        // set_state(name_eff, static_cast<double>(effort_rot));

        // // Latch force-closure when effort exceeds threshold
        // constexpr float EFFORT_THRESHOLD_NM = 0.75f;
        // if (std::abs(effort_rot) >= EFFORT_THRESHOLD_NM) {
        //     if (!this->gripper_force_closure_) {
        //         this->gripper_force_closure_counter_ += 1;
        //     }
        // } else {
        //     this->gripper_force_closure_ = false;
        //     this->gripper_force_closure_counter_ = 0;
        // }

        // if (this->gripper_force_closure_counter_ > 10) {
        //     this->gripper_hold_position_ = this->motors[6].Get_Position(); // in rot
        //     this->gripper_force_closure_ = true;
        // }

        return return_type::OK;
    }

    return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        return_type rt;

        rt = read_manipulator();
        if (rt != return_type::OK) {
            return rt;
        }

        rt = read_gripper();
        if (rt != return_type::OK) {
            return rt;
        }

        return return_type::OK;
    }

    
    return_type RobotSystem::write_manipulator()
    {
        Eigen::VectorXd tau_gravity;

        // gravity torques
        if (this->use_gravity_compensation_) 
        {
            Eigen::VectorXd q = pinocchio::neutral(pin_model_);
            
            // get current q
            for (std::size_t i=0; i < this->manipulator_joint_names_.size(); i++)
            {
                const auto name_pos = this->manipulator_joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION;
                q[i] = get_state(name_pos);    
            }
            
            // compute g(q)
            tau_gravity = pinocchio::computeGeneralizedGravity(this->pin_model_, this->pin_data_, q);
        }
        else  // no gravity compensation
        {
            tau_gravity = Eigen::VectorXd::Zero(this->manipulator_joint_names_.size());
        }


        // send commands
        for (std::size_t i=0; i < this->manipulator_joint_names_.size(); i++)
        {
            const auto joint_name = this->manipulator_joint_names_[i];

            const auto name_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
            const double pos    = get_command(name_pos);
            
            const auto name_vel = joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
            const double vel    = get_command(name_vel);

            const double tau_ff = tau_gravity[i];   // feed forward torque

            this->mc->control_mit(
                joint_name,
                this->motor_kp_[i],
                this->motor_kd_[i],
                pos,
                vel,
                tau_ff
            );
        }

        // read the response of each motor
        for (std::size_t i=0; i < this->manipulator_joint_names_.size(); i++)
        {
            this->mc->receive_motor_status();
        }

        return return_type::OK;
    }

    return_type RobotSystem::write_gripper()
    {
        // const auto name_pos = info_.joints[6].name + "/" + hardware_interface::HW_IF_POSITION;
        // float q7_lin = static_cast<float>(get_command(name_pos));
        // float q7_rot = q7_lin * this->gear_pinion_lin_to_rot;

        // // Clamp to valid rotational range
        // if (q7_rot < -5.0f || q7_rot > 0.0f) {
        //     q7_rot = std::clamp(q7_rot, -5.0f, 0.0f);
        // }

        // // If force closure is latched, hold the position where contact was made
        // // rather than continuing to push — this keeps grip without grinding
        // if (this->gripper_force_closure_) {
        //     q7_rot = this->gripper_hold_position_;
        // }

        // this->dm.control_mit(
        //     this->motors[6],
        //     this->motor_kp_[6],
        //     this->motor_kd_[6],
        //     q7_rot,
        //     0.0f,
        //     0.0f // tau
        // );

        return return_type::OK;
    }

    return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return_type rt;

        rt = write_manipulator();
        if (rt != return_type::OK) {
            return rt;
        }

        rt = write_gripper();
        if (rt != return_type::OK) {
            return rt;
        }
        
        return return_type::OK;
    }
    

}  // namespace damiao_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
damiao_hardware_interface::RobotSystem, hardware_interface::SystemInterface)
