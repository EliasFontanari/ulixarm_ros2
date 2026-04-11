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

        // input params
        // this->gear_pinion_module = info_.hardware_parameters.at("gear_pinion_module");
        // this->gear_pinion_number_teeth  = info_.hardware_parameters.at("gear_pinion_number_teeth");

        this->gear_pinion_module = 0.001f;
        this->gear_pinion_number_teeth = 14.0f;

        this->gear_pinion_rot_to_lin = (-1.0f)*(this->gear_pinion_module*this->gear_pinion_number_teeth) / 2.0f;
        this->gear_pinion_lin_to_rot = 1.0f / this->gear_pinion_rot_to_lin;

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

        // ATTENTION! overwrite the positions with actual ones
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

        // Load pinocchio model from URDF
        pinocchio::urdf::buildModel(urdf_path_, pin_model_);
        pin_data_ = pinocchio::Data(pin_model_);

        double total_mass = 0.0;
        for (int i = 0; i < pin_model_.nbodies; i++)
            total_mass += pin_model_.inertias[i].mass();
        RCLCPP_INFO(get_logger(), "Total mass seen by pinocchio: %.3f kg", total_mass);

        return CallbackReturn::SUCCESS;
    }

    return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        // rclcpp::Logger my_logger = rclcpp::get_logger("write_logger");

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


        // RCLCPP_INFO(my_logger, "\t\tq7_lin: %f\t\tq7_rot: %f\t\teffort: %f Nm",
        //             pos, this->motors[6].Get_Position(), effort_rot);


        return return_type::OK;
    }

    return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // rclcpp::Logger my_logger = rclcpp::get_logger("write_logger");

        // /*************************
        //  *      MANIPULATOR
        //  *************************/
        // for (std::size_t i = 0; i < (info_.joints.size()-1); i++)
        // {
        //     const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
        //     const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
        //     const float pos = static_cast<float> (get_command(name_pos));
        //     const float vel = static_cast<float> (get_command(name_vel));

        //     this->dm.control_mit(
        //       this->motors[i], 
        //       this->motor_kp[i], 
        //       this->motor_kd[i], 
        //       pos,
        //       vel,
        //       0.0f // tau
        //     );
        // }


        /*************************
         *      MANIPULATOR
         *************************/
        const std::size_t n_manip = info_.joints.size() - 1; // exclude gripper

        // --- Build q, v=0, a=0 vectors for pinocchio ---
        Eigen::VectorXd q = pinocchio::neutral(pin_model_);
        // Eigen::VectorXd v = Eigen::VectorXd::Zero(pin_model_.nv);
        // Eigen::VectorXd a = Eigen::VectorXd::Zero(pin_model_.nv);

        for (std::size_t i = 0; i < n_manip; i++)
        {
            const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            // Read current STATE position (not command) for gravity compensation
            q[i] = get_state(name_pos); // or hw_positions_[i] depending on your interface
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

            
            this->dm.control_mit(
                this->motors[i],
                this->motor_kp[i],
                this->motor_kd[i],
                pos,
                vel,
                tau_ff   // gravity compensation feedforward
            );
            // this->dm.control_mit(
            //     this->motors[i],
            //     0.0f,
            //     0.0f,
            //     0.0f,
            //     0.0f,
            //     tau_ff   // gravity compensation feedforward
            // );
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
            this->motor_kp[6],
            this->motor_kd[6],
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
