#include <termios.h>
#include <time.h>

#include <stdexcept>
#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "damiao_hardware_interface/error_codes.hpp"
#include "damiao_hardware_interface/motor.hpp"
#include "damiao_hardware_interface/motor_control.hpp"

namespace py = pybind11;

namespace
{
void throw_on_error(int rc, const char *action)
{
    if (rc < 0)
    {
        throw std::runtime_error(std::string(action) + ": " + error_message(static_cast<ErrorCode>(rc)));
    }
}
} // namespace

PYBIND11_MODULE(damiao_motor_control, m)
{
    m.doc() = "Python bindings for damiao motor control";

    py::enum_<damiao::DMMotorType>(m, "DMMotorType")
        .value("DM4340", damiao::DMMotorType::DM4340)
        .value("DM4310", damiao::DMMotorType::DM4310)
        .value("DMJ3507", damiao::DMMotorType::DMJ3507)
        .export_values();

    py::class_<damiao::MotorControl>(m, "MotorControl")
        .def(py::init<>())
        .def("init", [](damiao::MotorControl &self, const std::string &port, int baudrate, int timeout_ms) {
            const int rc = self.init(port.c_str(), static_cast<speed_t>(baudrate), static_cast<time_t>(timeout_ms));
            throw_on_error(rc, "init");
            return rc;
        })
        .def("add_motor", &damiao::MotorControl::add_motor,
             py::arg("motor_name"),
             py::arg("motor_type"),
             py::arg("slave_id"),
             py::arg("master_id"))
        .def("enable_motor", [](damiao::MotorControl &self, const std::string &motor_name) {
            const int rc = self.enable_motor(motor_name);
            throw_on_error(rc, "enable_motor");
            return rc;
        })
        .def("enable_motor_all", [](damiao::MotorControl &self) {
            const int rc = self.enable_motor_all();
            throw_on_error(rc, "enable_motor_all");
            return rc;
        })
        .def("disable_motor", [](damiao::MotorControl &self, const std::string &motor_name) {
            const int rc = self.disable_motor(motor_name);
            throw_on_error(rc, "disable_motor");
            return rc;
        })
        .def("disable_motor_all", [](damiao::MotorControl &self) {
            const int rc = self.disable_motor_all();
            throw_on_error(rc, "disable_motor_all");
            return rc;
        })
        .def("refresh_motor_status", [](damiao::MotorControl &self, const std::string &motor_name) {
            const int rc = self.refresh_motor_status(motor_name);
            throw_on_error(rc, "refresh_motor_status");
            return rc;
        })
        .def("refresh_motor_status_all", [](damiao::MotorControl &self) {
            const int rc = self.refresh_motor_status_all();
            throw_on_error(rc, "refresh_motor_status_all");
            return rc;
        })
        .def("get_position", [](damiao::MotorControl &self, const std::string &motor_name) {
            double position = 0.0;
            const int rc = self.get_position(motor_name, position);
            throw_on_error(rc, "get_position");
            return position;
        })
        .def("get_velocity", [](damiao::MotorControl &self, const std::string &motor_name) {
            double velocity = 0.0;
            const int rc = self.get_velocity(motor_name, velocity);
            throw_on_error(rc, "get_velocity");
            return velocity;
        })
        .def("get_torque", [](damiao::MotorControl &self, const std::string &motor_name) {
            double torque = 0.0;
            const int rc = self.get_torque(motor_name, torque);
            throw_on_error(rc, "get_torque");
            return torque;
        })
        .def("control_mit", [](damiao::MotorControl &self, const std::string &motor_name,
                               double kp, double kd, double q, double dq, double tau) {
            const int rc = self.control_mit(motor_name, kp, kd, q, dq, tau);
            throw_on_error(rc, "control_mit");
            return rc;
        })
        .def("receive_motor_data", [](damiao::MotorControl &self) {
            const int rc = self.receive_motor_data();
            throw_on_error(rc, "receive_motor_data");
            return rc;
        });
}
