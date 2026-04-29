#include "damiao_hardware_interface/damiao_hardware_interface.hpp"


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

        // Load gravity compensation flag
        {
            if (info_.hardware_parameters.find("use_gravity_compensation") != 
                info_.hardware_parameters.end())
            {
                this->use_gravity_compensation_ = 
                    info_.hardware_parameters.at("use_gravity_compensation") == "true";
            } else {
                this->use_gravity_compensation_ = true; // default
            }
            
            RCLCPP_INFO(get_logger(), "Gravity compensation: %s", 
                this->use_gravity_compensation_ ? "ENABLED" : "DISABLED");
        }

        // Load free floating flag
        {
            if (info_.hardware_parameters.find("use_free_floating") != 
                info_.hardware_parameters.end())
            {
                this->use_free_floating_ = 
                    info_.hardware_parameters.at("use_free_floating") == "true";
            } else {
                this->use_free_floating_ = false; // default
            }

            if (this->use_free_floating_ && !this->use_gravity_compensation_)
            {
                this->use_gravity_compensation_ = true;
                
                RCLCPP_WARN(get_logger(), 
                    "Switching gravity compensation if in free floating mode");
            }
            
            RCLCPP_INFO(get_logger(), "Free floating: %s", 
                this->use_free_floating_ ? "ENABLED" : "DISABLED");
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
                {
                    this->motor_kp_[i] = std::stod(joint.parameters.at("Kp"));
                }
                else 
                {
                    RCLCPP_ERROR(get_logger(), "Joint '%s' missing Kp!", joint.name.data());
                    return CallbackReturn::ERROR;
                }

    
                // Read Kd
                if (joint.parameters.find("Kd") != joint.parameters.end())
                {
                    this->motor_kd_[i] = std::stod(joint.parameters.at("Kd"));
                } else {
                    RCLCPP_ERROR(get_logger(), "Joint '%s' missing Kd!", joint.name.data());
                    return CallbackReturn::ERROR;
                }
                
                RCLCPP_INFO(get_logger(), "Joint '%s' — Kp: %.2f  Kd: %.2f",
                    joint.name.data(), motor_kp_[i], motor_kd_[i]);
                    
            }
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
        
        
        int rc;

        // Establish connection motor controller with serial
        {

            // Serial connection
            rc = this->mc.init(this->port_.data(), static_cast<speed_t>(this->baudrate_));
            if (rc < 0)    
                return CallbackReturn::ERROR;

            RCLCPP_INFO(get_logger(), "Initialized port: %s - baudrate: %d", this->port_.data(), this->baudrate_);
            
            // Initialize motors
            this->mc.add_motor(this->manipulator_joint_names_[0], damiao::DM4340,  0x01, 0x11);
            this->mc.add_motor(this->manipulator_joint_names_[1], damiao::DM4340,  0x02, 0x12);
            this->mc.add_motor(this->manipulator_joint_names_[2], damiao::DM4340,  0x03, 0x13);
            this->mc.add_motor(this->manipulator_joint_names_[3], damiao::DM4310,  0x04, 0x14);
            this->mc.add_motor(this->manipulator_joint_names_[4], damiao::DM4310,  0x05, 0x15);
            this->mc.add_motor(this->manipulator_joint_names_[5], damiao::DM4310,  0x06, 0x16);
            this->mc.add_motor(this->gripper_joint_names_[0],     damiao::DMJ3507, 0x07, 0x17);
            RCLCPP_INFO(get_logger(), "Added all motors to MotorController");
            
            // Enable motors
            RCLCPP_INFO(get_logger(), "Enabling motors...");
            
            rc = this->mc.enable_motor_all();
            if (rc < 0)
                return CallbackReturn::ERROR;

            RCLCPP_INFO(get_logger(), "Enabled all motors!");


            // Initial status refresh
            RCLCPP_INFO(get_logger(), "Refreshing motor status...");

            rc = this->mc.refresh_motor_status_all();
            if (rc < 0)
                return CallbackReturn::ERROR;

            RCLCPP_INFO(get_logger(), "Refreshed all motor status!");
        }

        // ATTENTION! overwrite the positions with ones from hardware sensors
        {
            double pos;

            // Manipulator
            for (const auto& joint_name : this->manipulator_joint_names_)
            {
                const auto name_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
                rc = this->mc.get_position(joint_name, pos);
                if (rc < 0)
                    return CallbackReturn::ERROR;
                set_state(name_pos, pos);
                set_command(name_pos, pos);
            }

            // Gripper
            for (const auto& joint_name : this->gripper_joint_names_)
            {
                const auto name_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
                rc = this->mc.get_position(joint_name, pos);
                if (rc < 0)
                    return CallbackReturn::ERROR;
                pos = pos * this->gear_pinion_rot_to_lin;
                set_state(name_pos, pos);
                set_command(name_pos, pos);
            }
        }

        return CallbackReturn::SUCCESS;
    }

    return_type RobotSystem::read_manipulator()
    {
        int rc;
        double pos;
        double vel;

        // read values from stored data, does not polls from actual hardware
        // polling to actual hardware happens after write, as motors respond after sending command
        for (const auto& joint_name : this->manipulator_joint_names_)
        {
            const auto name_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
            rc = this->mc.get_position(joint_name, pos);
            if (rc < 0)
                return return_type::ERROR;
            set_state(name_pos, pos);
            
            const auto name_vel = joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
            rc = this->mc.get_velocity(joint_name, vel);
            if (rc < 0)
                return return_type::ERROR;
            set_state(name_vel, vel);
        }

        return return_type::OK;
    }

    return_type RobotSystem::read_gripper()
    {
        int rc;
        double pos;
        double vel;
        double eff;

        // read values from stored data, does not polls from actual hardware
        // polling to actual hardware happens after write, as motors respond after sending command
        for (const auto& joint_name : this->gripper_joint_names_)
        {
            const auto name_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
            rc = this->mc.get_position(joint_name, pos);
            if (rc < 0)
                return return_type::ERROR;
            pos = pos * this->gear_pinion_rot_to_lin;
            set_state(name_pos, pos);
            
            const auto name_vel = joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
            rc = this->mc.get_velocity(joint_name, vel);
            if (rc < 0)
                return return_type::ERROR;
            vel = vel * this->gear_pinion_rot_to_lin;
            set_state(name_vel, vel);

            // the torque on the motor, NOT the force on the finger
            const auto name_eff = joint_name + "/" + hardware_interface::HW_IF_EFFORT;
            rc = this->mc.get_torque(joint_name, eff);
            if (rc < 0)
                return return_type::ERROR;
            eff = eff * this->gear_pinion_rot_to_lin;
            set_state(name_eff, eff);
        }

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
        // set gravity compensation
        Eigen::VectorXd tau_gravity;
        if (this->use_gravity_compensation_) 
        {
            Eigen::VectorXd q = pinocchio::neutral(pin_model_);
            
            // get current q
            for (size_t i=0; i < this->manipulator_joint_names_.size(); i++)
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
        
        // set free floating gains
        std::vector<double> kp;
        std::vector<double> kd;
        const size_t n_motors = this->manipulator_joint_names_.size() + this->gripper_joint_names_.size();
        kp.resize(n_motors);
        kd.resize(n_motors);
        
        for (size_t i=0; i<n_motors; i++)
        {
            if (this->use_free_floating_)
            {
                kp[i] = 0.0;
                kd[i] = this->motor_kd_[i] * 0.1;
            }
            else
            {
                kp[i] = this->motor_kp_[i];
                kd[i] = this->motor_kd_[i];
            }
        }

        // send commands
        int rc;

        for (size_t i=0; i < this->manipulator_joint_names_.size(); i++)
        {
            const auto joint_name = this->manipulator_joint_names_[i];

            const auto name_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
            const double pos = get_command(name_pos);
            
            const auto name_vel = joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
            const double vel = get_command(name_vel);

            const double tau_ff = tau_gravity[i];   // feed forward torque

            rc = this->mc.control_mit(
                joint_name,
                kp[i],
                kd[i],
                pos,
                vel,
                tau_ff
            );

            if (rc < 0) 
                return return_type::ERROR;
        }

        return return_type::OK;
    }

    return_type RobotSystem::write_gripper()
    {
    //     int rc = 0;

    //     const size_t motor_kx_offset = this->manipulator_joint_names_.size();
    //     const auto joint_name = this->gripper_joint_names_[0];

    //     const auto name_pos = joint_name + "/" + hardware_interface::HW_IF_POSITION;
    //     const auto name_vel = joint_name + "/" + hardware_interface::HW_IF_VELOCITY;

    //     const double goal_pos_lin  = get_command(name_pos);
    //     const double state_pos_lin = get_state(name_pos);
    //     const double state_pos_rot = state_pos_lin * this->gear_pinion_lin_to_rot;
    //     const double state_vel_lin = get_state(name_vel);

    //     const double goal_pos_rot_cmd = goal_pos_lin * this->gear_pinion_lin_to_rot;

    //     // ----------------------------
    //     // thresholds
    //     // ----------------------------
    //     const double pos_threshold_far = 0.001;
    //     const double vel_threshold     = 0.001;
    //     const double stall_timeout     = 0.1;

    //     const bool closing_cmd = std::abs(goal_pos_lin) < 1e-4;
    //     const bool opening_cmd = !closing_cmd;

    //     const bool far_from_zero = state_pos_lin > pos_threshold_far;
    //     const bool low_velocity  = std::abs(state_vel_lin) < vel_threshold;

    //     // ----------------------------
    //     // stall timer
    //     // ----------------------------
    //     if (closing_cmd && far_from_zero && low_velocity)
    //     {
    //         if (!stall_timer_active_)
    //         {
    //             stall_timer_active_ = true;
    //             stall_start_time_ = get_clock()->now();
    //         }
    //     }
    //     else
    //     {
    //         stall_timer_active_ = false;
    //     }

    //     const bool stalled =
    //         closing_cmd &&
    //         far_from_zero &&
    //         stall_timer_active_ &&
    //         (get_clock()->now() - stall_start_time_).seconds() >= stall_timeout;

    //     // ----------------------------
    //     // Latch logic (IMPORTANT CHANGE)
    //     // ----------------------------
    //     if (stalled && !stall_latched_)
    //     {
    //         RCLCPP_INFO(get_logger(), "Stall confirmed → latching goal");

    //         latched_goal_pos_rot_ = state_pos_rot - 0.04;
    //         stall_latched_ = true;
    //     }

    //     // reset latch if opening
    //     if (opening_cmd)
    //     {
    //         stall_latched_ = false;
    //     }

    //     // ----------------------------
    //     // control logic
    //     // ----------------------------
    //     if (closing_cmd)
    //     {
    //         if (stall_latched_)
    //         {
    //             // HOLD force closure target (DO NOT recompute)
    //             rc = this->mc.control_mit(
    //                 joint_name,
    //                 this->motor_kp_[motor_kx_offset],
    //                 this->motor_kd_[motor_kx_offset],
    //                 latched_goal_pos_rot_,
    //                 0.0,
    //                 0.0
    //             );
    //         }
    //         else if (far_from_zero)
    //         {
    //             // constant velocity closing (FIXED SIGN AS REQUESTED)
    //             const double closing_velocity = 0.3;

    //             rc = this->mc.control_mit(
    //                 joint_name,
    //                 0.0,
    //                 this->motor_kd_[motor_kx_offset],
    //                 0.0,
    //                 closing_velocity,
    //                 0.0
    //             );
    //         }
    //         else
    //         {
    //             // near goal → position control to 0
    //             rc = this->mc.control_mit(
    //                 joint_name,
    //                 this->motor_kp_[motor_kx_offset],
    //                 this->motor_kd_[motor_kx_offset],
    //                 0.0,
    //                 0.0,
    //                 0.0
    //             );
    //         }
    //     }
    //     else if (opening_cmd)
    //     {
    //         rc = this->mc.control_mit(
    //             joint_name,
    //             this->motor_kp_[motor_kx_offset],
    //             this->motor_kd_[motor_kx_offset],
    //             goal_pos_rot_cmd,
    //             0.0,
    //             0.0
    //         );
    //     }

    //     if (rc < 0) return return_type::ERROR;
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
