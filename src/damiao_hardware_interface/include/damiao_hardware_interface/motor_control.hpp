#ifndef ULIXARM_HARDWARE__MOTOR_CONTROL_HPP_
#define ULIXARM_HARDWARE__MOTOR_CONTROL_HPP_

#include <utility>
#include <variant>
#include <cstdint>
#include <cmath>
#include <vector>
#include <array>
#include <unordered_map>
#include <string>

#include "damiao_hardware_interface/motor.hpp"
#include "damiao_hardware_interface/error_codes.hpp"
#include "damiao_hardware_interface/serial_port.hpp"


#pragma pack(push, 1)

struct CANReceiveFrame
{
    uint8_t frame_header;
    uint8_t cmd;                // Command  0x00: heartbeat
                                //          0x01: receive fail  0x11: receive success
                                //          0x02: send fail     0x12: send success
                                //          0x03: set baudrate fail 0x13: set baudrate success
                                //          0xEE: communication error, format field contains error code
                                //          0x08: overvoltage
                                //          0x09: undervoltage
                                //          0x0A: overcurrent
                                //          0x0B: MOS overtemperature
                                //          0x0C: motor coil overtemperature
                                //          0x0D: communication loss
                                //          0x0E: overload
    uint8_t can_data_len: 6;    // data length
    uint8_t can_ide: 1;         // 0: standard frame    1: extended frame
    uint8_t can_rtr: 1;         // 0: data frame        1: remote frame
    uint32_t can_id;            // ID fed back from the motor
    uint8_t can_data[8];
    uint8_t frame_end;          // end of frame
};

struct CANSendFrame
{
    uint8_t frame_header[2] = {0x55, 0xAA};     // frame header
    uint8_t frame_len = 0x1e;                   // frame length
    uint8_t cmd = 0x03;             // command: 1 = forward CAN data frame
                                    //          2 = PC–device handshake, device responds OK
                                    //          3 = non-acknowledged CAN forward, no send-status feedback
    uint32_t send_times = 1;        // number of times to send
    uint32_t time_interval = 10;    // time interval (ms)       NOTE: not used for send_times = 1
    uint8_t id_type = 0;            // ID type: 0 = standard frame  1 = extended frame
    uint32_t can_id = 0x01;         // CAN ID — uses the motor ID as the CAN ID
    uint8_t frame_type = 0;         // frame type: 0 = data frame  1 = remote frame
    uint8_t len = 0x08;             // payload length
    uint8_t id_acc = 0;
    uint8_t data_acc = 0;
    uint8_t data[8] = {0};
    uint8_t crc = 0;                // not parsed — any value accepted
 
    void prepare(const damiao::MotorID id, const uint8_t* send_data)
    {
        can_id = id;
        std::copy(send_data, send_data+8, data);
    }
};

#pragma pack(pop)

namespace damiao
{

    class MotorControl
    {
    public:
    
        MotorControl(){};
        ~MotorControl(){};
    
        int init(const char* port, speed_t baudrate);
    
        void add_motor(const std::string motor_name, const damiao::DMMotorType motor_type, 
            const damiao::MotorID slave_id, const damiao::MotorID master_id);
        
        int enable_motor(const std::string motor_name);
        int enable_motor_all();
    
        int disable_motor(const std::string motor_name);
        int disable_motor_all();
    
        int refresh_motor_status(const std::string motor_name);
        int refresh_motor_status_all();
    
        int get_position(std::string motor_name, double& position);
        int get_velocity(std::string motor_name, double& velocity);
        int get_torque(std::string motor_name, double& torque);
    
        int control_mit(const std::string motor_name, double kp, double kd, double q, double dq, double tau);
        
        private:
        
        uint16_t double_to_uint(double x, double xmin, double xmax, uint8_t bits);
        double uint_to_double(uint16_t x, double xmin, double xmax, uint8_t bits);
        
        int send_motor_data(uint8_t slave_id, const std::array<uint8_t,8>& data_buf);
        int unpack_motor_data(CANReceiveFrame* receive_data);
        int receive_motor_data();
        int send_control_cmd(damiao::MotorID id , uint8_t cmd);
    
        damiao::SerialPort serial_;
        std::unordered_map<std::string, Motor> motors_;
        std::unordered_map<damiao::MotorID, std::string> lut_master_id_to_motor_name_;
    };
    
} // namespace damiao




#endif // ULIXARM_HARDWARE__MOTOR_CONTROL_HPP_