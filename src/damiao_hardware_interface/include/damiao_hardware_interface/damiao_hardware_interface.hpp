// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ULIXARM_HARDWARE__DAMIAO_HARDWARE_INTERFACE_HPP_
#define ULIXARM_HARDWARE__DAMIAO_HARDWARE_INTERFACE_HPP_

#include "string"
#include "unordered_map"
#include "vector"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <memory>

// #include "damiao.h"
#include "damiao_hardware_interface/motor_control.hpp"


using hardware_interface::return_type;

namespace damiao_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobotSystem : public hardware_interface::SystemInterface
{
    public:

    CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    
    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
    
    protected:

    return_type read_manipulator();
    return_type read_gripper();
    
    return_type write_manipulator();
    return_type write_gripper();

    std::vector<std::string> manipulator_joint_names_;
    std::vector<std::string> gripper_joint_names_;
    
    // motor control
    damiao::MotorControl mc;
    
    // serial port
    std::string port_;
    int baudrate_;
    
    // mit controller
    std::vector<double> motor_kp_;
    std::vector<double> motor_kd_;
    std::vector<double> motor_kd_limits = {
        20.0, 400.0,
        20.0, 400.0,
        20.0, 400.0,
        10.0, 200.0,
        10.0, 200.0,
        10.0, 200.0
    };
    
    // gripper kinematics
    const double gear_pinion_rot_to_lin_ = -0.007;
    const double gear_pinion_lin_to_rot_ = -142.857142857;

    // pinocchio
    pinocchio::Model pin_model_;
    pinocchio::Data  pin_data_;

    // interactive flags
    bool use_gravity_compensation_;
    bool use_free_floating_;

};

}  // namespace damiao_hardware_interface

#endif  // ULIXARM_HARDWARE__DAMIAO_HARDWARE_INTERFACE_HPP_
