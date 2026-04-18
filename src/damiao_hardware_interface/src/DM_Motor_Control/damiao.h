#ifndef DAMIAO_H
#define DAMIAO_H

#include "SerialPort.h"
#include <cmath>
#include <utility>
#include <vector>
#include <unordered_map>
#include <array>
#include <variant>
#include <cstdint>
#include <cmath>

namespace damiao
{
#pragma pack(1)
#define MotorID uint32_t

enum DMMotorType
{
    DM4310,
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM8006,
    DM8009,
    DM10010L,
    DM10010,
    DMH3510,
    DMH6215,
    DMG6220,
    DMJ3507,
    NUM_OF_MOTORS
};

typedef struct CANReceiveFrame
{
    uint8_t FrameHeader;
    uint8_t CMD;
    uint8_t canDataLen: 6;
    uint8_t canIde: 1;
    uint8_t canRtr: 1;
    uint32_t canId;
    uint8_t canData[8];
    uint8_t frameEnd;
};

typedef struct CANSendFrame
{
    uint8_t FrameHeader[2] = {0x55, 0xAA};
    uint8_t FrameLen = 0x1e;
    uint8_t CMD = 0x03;
    uint32_t sendTimes = 1;
    uint32_t timeInterval = 10;
    uint8_t IDType = 0;
    uint32_t canId=0x01;
    uint8_t frameType = 0;
    uint8_t len = 0x08;
    uint8_t idAcc=0;
    uint8_t dataAcc=0;
    uint8_t data[8]={0};
    uint8_t crc=0;

    void prepare(const MotorID id, const uint8_t* send_data)
    {
        canId = id;
        std::copy(send_data, send_data+8, data);
    }

};

#pragma pack()

typedef struct LimitParam
{
    double q;
    double dq;
    double tau;
};

LimitParam limit_param[NUM_OF_MOTORS] =
{
    {12.5, 30, 10 }, // DM4310
    {12.5, 50, 10 }, // DM4310_48V
    {12.5, 8, 27 },  // DM4340
    {12.5, 10, 27 }, // DM4340_48V
    {12.5, 45, 20 }, // DM6006
    {12.5, 45, 40 }, // DM8006
    {12.5, 45, 54 }, // DM8009
    {12.5,25,  200}, // DM10010L
    {12.5,20, 200},  // DM10010
    {12.5,280,1},    // DMH3510
    {12.5,45,10},    // DMH6215
    {12.5,45,10},     // DMG6220
    {12.5, 8, 3}     // DMJ3507
};

class Motor
{
public:

    Motor(DMMotorType motor_type, MotorID slave_id, MotorID master_id) 
    {
        this->motor_type_ = motor_type;
        this->slave_id_ = slave_id;
        this->master_id_ = master_id;
        this->limit_param_ = damiao::limit_param[motor_type];
    }

    DMMotorType get_motor_type() const { return this->motor_type_; }

    MotorID get_master_id() const { return this->master_id_; }
    MotorID get_slave_id()  const { return this->slave_id_; }

    double get_position() const { return this->state_q_; }
    void set_position(double q) { this->state_q_ = q; }
    
    double get_velocity() const { return this->state_dq_; }
    void set_velocity(double dq) { this->state_dq_ = dq; }
    
    double get_torque() const { return this->state_tau_; }
    void set_torque(double tau) { this->state_tau_ = tau; }

    double get_trot() const { return this->state_trot_; }
    double get_tmos() const { return this->state_tmos_; }

    LimitParam get_limit_param() { return this->limit_param_; }

private:
    MotorID slave_id_;
    MotorID master_id_;
    DMMotorType motor_type_;

    LimitParam limit_param_{};

    double state_q_  = 0.0f;
    double state_dq_ = 0.0f;
    double state_tau_= 0.0f;
    double state_trot_ = 0.0f;
    double state_tmos_ = 0.0f;
};


class MotorControl
{
    public:

    MotorControl(std::string port, speed_t baudrate)
        : serial_(port, baudrate)
    {
        return;
    }

    ~MotorControl() 
    {
        return;
    }

    void add_motor(const std::string motor_name, const DMMotorType motor_type, 
        const MotorID slave_id, const MotorID master_id)
    {
        Motor motor(motor_type, slave_id, master_id);
        this->motors_[motor_name] = motor;
        this->lut_slave_id_to_motor_name_[slave_id] = motor_name;
        return;
    }
    
    void enable_motor(const std::string motor_name)
    {
        auto it = this->motors_.find(motor_name);
        if (it != this->motors_.end()) {
            MotorID master_id = it->second.get_master_id();
        } else {
            throw std::runtime_error("Name does not exist!");
            return;
        }

        this->send_control_cmd(master_id, 0xFC);
        return;
    }

    void enable_motor_all()
    {
        for (const auto& [name, motor] : this->motors_) 
        {
            this->enable_motor(name);
        }
        return;
    }

    void disable_motor(const std::string motor_name)
    {
        auto it = this->motors_.find(motor_name);
        if (it != this->motors_.end()) {
            MotorID master_id = it->second.get_master_id();
        } else {
            throw std::runtime_error("Name does not exist!");
            return;
        }

        this->send_control_cmd(master_id, 0xFD);
        return;
    }

    void disable_motor_all()
    {
        for (const auto& [name, motor] : this->motors_) 
        {
            this->disable_motor(name);
        }
        return;
    }
    
    void refresh_motor_status(const std::string motor_name)
    {
        auto it = this->motors_.find(motor_name);
        if (it != this->motors_.end()) {
            MotorID slave_id = it->second.get_slave_id();
            MotorID master_id = it->second.get_master_id();
        } else {
            throw std::runtime_error("Name does not exist!");
        }
        
        uint32_t id = 0x7FF;
        
        uint8_t can_low = slave_id & 0xff; // id low 8 bit
        uint8_t can_high = (master_id >> 8) & 0xff; //id high 8 bit
        std::array<uint8_t, 8> data_buf = {can_low,can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_data.prepare(id, data_buf.data());
        this->serial_.write((uint8_t*)&send_data, sizeof(CANSendFrame));
        this->read_motor_status();
    }

    void refresh_motor_status_all()
    {
        for (const auto& [name, motor] : this->motors_) 
        {
            this->refresh_motor_status(name);
        }
        return;
    }
    
    void control_mit(const std::string motor_name, double kp, double kd, double q, double dq, double tau)
    {
        // check and get motor from map
        auto it = this->motors_.find(motor_name);
        if (it != this->motors_.end()) {
            MotorID slave_id = it->second.get_slave_id();
            LimitParam limit_param_cmd = it->second.get_limit_param();
        } else {
            throw std::runtime_error("Name does not exist!");
        }

        // map linearly to given bounds
        uint16_t kp_uint    = double_to_uint(kp, 0, 500, 12);
        uint16_t kd_uint    = double_to_uint(kd, 0, 5, 12);
        uint16_t q_uint     = double_to_uint(q,   -limit_param_cmd.q,   limit_param_cmd.q,   16);
        uint16_t dq_uint    = double_to_uint(dq,  -limit_param_cmd.dq,  limit_param_cmd.dq,  12);
        uint16_t tau_uint   = double_to_uint(tau, -limit_param_cmd.tau, limit_param_cmd.tau, 12);

        // pack data
        std::array<uint8_t, 8> data_buf{};
        data_buf[0] = (q_uint >> 8) & 0xff;
        data_buf[1] = q_uint & 0xff;
        data_buf[2] = dq_uint >> 4;
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
        data_buf[4] = kp_uint & 0xff;
        data_buf[5] = kd_uint >> 4;
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
        data_buf[7] = tau_uint & 0xff;

        // pack CAN frame
        CANSendFrame send_data;
        send_data.prepare(slave_id, data_buf.data());

        // send to data
        this->serial_.write((uint8_t*)&send_data, sizeof(CANSendFrame));

        return;
    }

    void read_motor_status()
    {
        CANReceiveFrame receive_data;

        int rc = this->serial_.read((uint8_t*)&receive_data, 0xAA, sizeof(CANReceiveFrame));
        if (rc <= 0) {
            return;
        }

        if(receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
        {
            auto & data = receive_data.canData;

            auto it = this->lut_slave_id_to_motor_name_.find(receive_data.canId);
            if (it != this->lut_slave_id_to_motor_name_.end()) {
                std::string motor_name = it->second;
            } else {
                return; // handle error
            }

            Motor motor = this->motors_.at(motor_name);
            LimitParam limit_param_receive = motor.get_limit_param();

            uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
            uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
            uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];
            double receive_q = uint_to_double(q_uint, -limit_param_receive.q, limit_param_receive.q, 16);
            double receive_dq = uint_to_double(dq_uint, -limit_param_receive.dq, limit_param_receive.dq, 12);
            double receive_tau = uint_to_double(tau_uint, -limit_param_receive.tau, limit_param_receive.tau, 12);
            
            motor.set_position(receive_q);
            motor.set_velocity(receive_dq);
            motor.set_tau(receive_tau);
        }

        return;
    }

    private:

    /************************
     *        UTILS
     ************************/
    uint16_t double_to_uint(double x, double xmin, double xmax, uint8_t bits) 
    {
        double span = xmax - xmin;
        double data_norm = (x - xmin) / span;
        uint16_t data_uint = data_norm * ((1 << bits) - 1);
        return data_uint;
    }

    double uint_to_double(uint16_t x, double xmin, double xmax, uint8_t bits)
    {
        double span = xmax - xmin;
        double data_norm = double(x) / ((1 << bits) - 1);
        double data = data_norm * span + xmin;
        return data;
    }

    void send_control_cmd(MotorID id , uint8_t cmd)
    {
        // pack data with cmd
        std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
        
        // pack CAN frame
        CANSendFrame send_data;
        send_data.prepare(id, data_buf.data());
        
        // send frame
        this->serial_.write((uint8_t*)&send_data, sizeof(CANSendFrame));
    }


    // 
    SerialPort serial_;
    std::unordered_map<std::string, Motor> motors_;
    std::unordered_map<MotorID, std::string> lut_slave_id_to_motor_name_;
};

};

#endif