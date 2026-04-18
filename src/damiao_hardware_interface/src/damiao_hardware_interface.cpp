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

        // Read robot_description
        auto node = rclcpp::Node::make_shared("hf_robot_descritption_listener");

        node->declare_parameter("robot_description", std::string(""));
        std::string urdf_string = node->get_parameter("robot_description")
                                    .get_value<std::string>();

        if (urdf_string.empty()) {
            RCLCPP_ERROR(get_logger(), "robot_description is empty!");
            return CallbackReturn::ERROR;
        }
        
        pinocchio::urdf::buildModelFromXML(urdf_string, pin_model_);
        pin_data_ = pinocchio::Data(pin_model_);


        // import params from file
        motor_kp_.resize(info_.joints.size());
        motor_kd_.resize(info_.joints.size());

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            const auto & joint = info_.joints[i];

            // Read Kp
            if (joint.parameters.find("Kp") != joint.parameters.end())
                motor_kp_[i] = static_cast<float>(std::stod(joint.parameters.at("Kp")));
            else {
                RCLCPP_ERROR(get_logger(), "Joint '%s' missing Kp!", joint.name.c_str());
                return CallbackReturn::ERROR;
            }

            // Read Kd
            if (joint.parameters.find("Kd") != joint.parameters.end())
                motor_kd_[i] = static_cast<float>(std::stod(joint.parameters.at("Kd")));
            else {
                RCLCPP_ERROR(get_logger(), "Joint '%s' missing Kd!", joint.name.c_str());
                return CallbackReturn::ERROR;
            }

            RCLCPP_INFO(get_logger(), "Joint '%s' — Kp: %.2f  Kd: %.2f",
                joint.name.c_str(), motor_kp_[i], motor_kd_[i]);
        }


        // gravity compensation flag
        use_gravity_compensation_ = true; // default

        if (info_.hardware_parameters.find("use_gravity_compensation") != 
            info_.hardware_parameters.end())
        {
            use_gravity_compensation_ = 
                info_.hardware_parameters.at("use_gravity_compensation") == "true";
        }
        
        
        // free floating flag
        use_free_floating_ = false; // default
        
        if (info_.hardware_parameters.find("use_free_floating") != 
            info_.hardware_parameters.end())
        {
            use_free_floating_ = 
                info_.hardware_parameters.at("use_free_floating") == "true";
            if (use_free_floating_) {
                use_gravity_compensation_ = true;
            }
        }

        RCLCPP_INFO(get_logger(), "Gravity compensation: %s", 
            use_gravity_compensation_ ? "ENABLED" : "DISABLED");

        RCLCPP_INFO(get_logger(), "Free Floating: %s", 
            use_free_floating_ ? "ENABLED" : "DISABLED");


        // open serial port
        const std::string port = info_.hardware_parameters.at("serial_port");
        const int baud_rate = std::stoi(info_.hardware_parameters.at("serial_baud_rate"));
        serial = std::make_shared<SerialPort>(port, baud_rate);        

        // Initialize motor controller with serial
        dm = damiao::Motor_Control(serial);

        // Initialize motors
        motors = {
            {damiao::DM4340, 0x01, 0x11},
            {damiao::DM4340, 0x02, 0x12},
            {damiao::DM4340, 0x03, 0x13},
            {damiao::DM4310, 0x04, 0x14},
            {damiao::DM4310, 0x05, 0x15},
            {damiao::DM4310, 0x06, 0x16},
            {damiao::DMJ3507,0x07, 0x17}
        };

        for (auto &motor : this->motors) {
            this->dm.addMotor(&motor);
            this->dm.enable(motor);
            this->dm.refresh_motor_status(motor);
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

        // ATTENTION! overwrite the positions with ones from hardware sensors
        for (std::size_t i = 0; i < (info_.joints.size()-1); i++)
        {
            const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            set_state(name_pos, static_cast<double>(this->motors[i].Get_Position()));
            set_command(name_pos, static_cast<double>(this->motors[i].Get_Position()));
        }

        const auto name_pos = info_.joints[6].name + "/" + hardware_interface::HW_IF_POSITION;
        const float pos = this->motors[6].Get_Position() * this->gear_pinion_rot_to_lin;
        set_state(name_pos, static_cast<double>(pos));
        set_command(name_pos, static_cast<double>(pos));

        return CallbackReturn::SUCCESS;
    }

    return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        /*************************
         *      MANIPULATOR
         *************************/
        for (std::size_t i = 0; i < (info_.joints.size()-1); i++)
        {
            const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            const double pos = static_cast<double>(this->motors[i].Get_Position());
            set_state(name_pos, pos);
            
            const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
            const double vel = static_cast<double>(this->motors[i].Get_Velocity());
            set_state(name_vel, vel);
        }

        // /*************************
        //  *        GRIPPER
        //  *************************/
        // const auto name_pos = info_.joints[6].name + "/" + hardware_interface::HW_IF_POSITION;
        // const float pos = this->motors[6].Get_Position() * this->gear_pinion_rot_to_lin;
        // set_state(name_pos, static_cast<double>(pos));
        
        // const auto name_vel = info_.joints[6].name + "/" + hardware_interface::HW_IF_VELOCITY;
        // const float vel = this->motors[6].Get_Velocity() * this->gear_pinion_rot_to_lin;
        // set_state(name_vel, static_cast<double>(vel));

        // RCLCPP_INFO(my_logger, "\t\tq7_lin: %f\t\tq7_rot: %f", pos, this->motors[6].Get_Position());

        /*************************
         *        GRIPPER
         *************************/
        const auto name_pos = info_.joints[6].name + "/" + hardware_interface::HW_IF_POSITION;
        const float pos = this->motors[6].Get_Position() * this->gear_pinion_rot_to_lin;
        set_state(name_pos, static_cast<double>(pos));

        const auto name_vel = info_.joints[6].name + "/" + hardware_interface::HW_IF_VELOCITY;
        const float vel = this->motors[6].Get_Velocity() * this->gear_pinion_rot_to_lin;
        set_state(name_vel, static_cast<double>(vel));

        const auto name_eff = info_.joints[6].name + "/" + hardware_interface::HW_IF_EFFORT;
        const float effort_rot = this->motors[6].Get_tau();
        set_state(name_eff, static_cast<double>(effort_rot));

        // Latch force-closure when effort exceeds threshold
        constexpr float EFFORT_THRESHOLD_NM = 0.75f;
        if (std::abs(effort_rot) >= EFFORT_THRESHOLD_NM) {
            if (!this->gripper_force_closure_) {
                this->gripper_force_closure_counter_ += 1;
            }
        } else {
            this->gripper_force_closure_ = false;
            this->gripper_force_closure_counter_ = 0;
        }

        if (this->gripper_force_closure_counter_ > 10) {
            this->gripper_hold_position_ = this->motors[6].Get_Position(); // in rot
            this->gripper_force_closure_ = true;
        }

        return return_type::OK;
    }

    return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // rclcpp::Logger my_logger = rclcpp::get_logger("write_logger");

        /*************************
         *      MANIPULATOR
         *************************/
        const std::size_t n_manip = info_.joints.size() - 1; // exclude gripper

        // --- Build q, v=0, a=0 vectors for pinocchio ---
        Eigen::VectorXd q = pinocchio::neutral(pin_model_);

        for (std::size_t i = 0; i < n_manip; i++)
        {
            const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            q[i] = get_state(name_pos);
        }

        // --- Compute gravity torques: g(q) = RNEA(q, 0, 0) ---
        const Eigen::VectorXd & tau_gravity = pinocchio::computeGeneralizedGravity(
            pin_model_, pin_data_, q);

        // --- Send commands ---
        for (std::size_t i = 0; i < n_manip; i++)
        {
            const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;

            const float pos     = static_cast<float>(get_command(name_pos));
            const float vel     = static_cast<float>(get_command(name_vel));
            const float tau_ff  = static_cast<float>(tau_gravity[i]);

            if (use_gravity_compensation_ && !use_free_floating_) {
                this->dm.control_mit(
                    this->motors[i],
                    this->motor_kp_[i],
                    this->motor_kd_[i],
                    pos,
                    vel,
                    tau_ff   // gravity compensation feedforward
                );
            } else if (use_gravity_compensation_ && use_free_floating_) {
                this->dm.control_mit(
                    this->motors[i],
                    0.0f,
                    0.2f*this->motor_kd_[i],
                    0.0f,
                    0.0f,
                    tau_ff   // gravity compensation feedforward
                );
            } else {
                this->dm.control_mit(
                    this->motors[i],
                    this->motor_kp_[i],
                    this->motor_kd_[i],
                    pos,
                    vel,
                    0.0f
                );
            }       

        }

        // /*************************
        //  *        GRIPPER
        //  *************************/
        // const auto name_pos = info_.joints[6].name + "/" + hardware_interface::HW_IF_POSITION;
        // float q7_lin = static_cast<float> (get_command(name_pos));
        // float q7_rot = q7_lin * this->gear_pinion_lin_to_rot;
        // if (q7_rot < -5.0f or q7_rot > 0.0f) {
        //     return_type::OK;
        // }

        // this->dm.control_mit(
        //     this->motors[6], 
        //     this->motor_kp[6], 
        //     this->motor_kd[6], 
        //     q7_rot,
        //     0.0f,
        //     0.0f // tau
        // );


        /*************************
         *        GRIPPER
         *************************/
        const auto name_pos = info_.joints[6].name + "/" + hardware_interface::HW_IF_POSITION;
        float q7_lin = static_cast<float>(get_command(name_pos));
        float q7_rot = q7_lin * this->gear_pinion_lin_to_rot;

        // Clamp to valid rotational range
        if (q7_rot < -5.0f || q7_rot > 0.0f) {
            q7_rot = std::clamp(q7_rot, -5.0f, 0.0f);
        }

        // If force closure is latched, hold the position where contact was made
        // rather than continuing to push — this keeps grip without grinding
        if (this->gripper_force_closure_) {
            q7_rot = this->gripper_hold_position_;
        }

        this->dm.control_mit(
            this->motors[6],
            this->motor_kp_[6],
            this->motor_kd_[6],
            q7_rot,
            0.0f,
            0.0f // tau
        );
        
        return return_type::OK;
    }

}  // namespace damiao_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
damiao_hardware_interface::RobotSystem, hardware_interface::SystemInterface)
