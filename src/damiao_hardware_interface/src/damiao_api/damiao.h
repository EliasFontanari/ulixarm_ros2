#ifndef DAMIAO_H
#define DAMIAO_H

#include "serial_port.h"
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

using MotorID = uint32_t;

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
 
    void prepare(const MotorID id, const uint8_t* send_data)
    {
        can_id = id;
        std::copy(send_data, send_data+8, data);
    }
};

#pragma pack(pop)

struct LimitParam
{
    double q;
    double dq;
    double tau;
};

enum DMMotorType
{
    DM4340,
    DM4310,
    DMJ3507,
    NUM_OF_MOTORS
};


LimitParam limit_params[NUM_OF_MOTORS] = {
    {12.5, 8.0, 28.0 },  // DM4340
    {12.5, 30.0, 10.0 }, // DM4310
    {12.5, 15.0, 3.0}     // DMJ3507
};

class Motor
{
public:

    Motor(DMMotorType motor_type, MotorID slave_id, MotorID master_id) : 
    motor_type_(motor_type), 
    slave_id_(slave_id), 
    master_id_(master_id), 
    limit_param_(damiao::limit_params[motor_type])
    {}

    Motor() = default;

    DMMotorType get_motor_type() const { return this->motor_type_; }

    MotorID get_master_id() const { return this->master_id_; }
    MotorID get_slave_id()  const { return this->slave_id_; }

    double  get_position() const { return this->state_q_; }
    void    set_position(double q) { this->state_q_ = q; }
    
    double  get_velocity() const { return this->state_dq_; }
    void    set_velocity(double dq) { this->state_dq_ = dq; }
    
    double  get_torque() const { return this->state_tau_; }
    void    set_torque(double tau) { this->state_tau_ = tau; }

    uint8_t  get_trot() const { return this->state_trot_; }
    void    set_trot(uint8_t trot) { this->state_trot_ = trot; }
    
    uint8_t  get_tmos() const { return this->state_tmos_; }
    void    set_tmos(uint8_t tmos) { this->state_tmos_ = tmos; }
    
    LimitParam get_limit_param() const { return this->limit_param_; }

private:
    MotorID slave_id_;
    MotorID master_id_;
    DMMotorType motor_type_;

    LimitParam limit_param_{};

    double state_q_  = 0.0;
    double state_dq_ = 0.0;
    double state_tau_= 0.0;
    uint8_t state_trot_ = 0;
    uint8_t state_tmos_ = 0;
};


class MotorControl
{
    public:

    MotorControl(const char* port, speed_t baudrate){
        this->serial_.emplace(port, baudrate);
    }
    
    ~MotorControl(){};
    
    void add_motor(const std::string motor_name, const DMMotorType motor_type, 
        const MotorID slave_id, const MotorID master_id)
    {
        Motor motor(motor_type, slave_id, master_id);
        this->motors_[motor_name] = motor;
        this->lut_master_id_to_motor_name_[master_id] = motor_name;
        return;
    }
    
    void enable_motor(const std::string motor_name)
    {
        auto it = this->motors_.find(motor_name);
        if (it != this->motors_.end()) {
            MotorID slave_id = it->second.get_slave_id();
            this->send_control_cmd(slave_id, 0xFC);
        } else {
            throw std::runtime_error("Name does not exist!");
        }

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
            MotorID slave_id = it->second.get_slave_id();
            this->send_control_cmd(slave_id, 0xFD);
        } else {
            throw std::runtime_error("Name does not exist!");
        }

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
        MotorID slave_id;

        auto it = this->motors_.find(motor_name);
        if (it != this->motors_.end()) {
            slave_id = it->second.get_slave_id();
        } else {
            throw std::runtime_error("Name does not exist!");
            return;
        }
        
        uint32_t id = 0x7FF;
        
        uint8_t can_low = slave_id & 0xff; // id low 8 bit
        uint8_t can_high = (slave_id >> 8) & 0xff; //id high 8 bit
        std::array<uint8_t, 8> data_buf = {can_low,can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
        
        CANSendFrame send_data;
        send_data.prepare(id, data_buf.data());
        this->serial_->write((uint8_t*)&send_data, sizeof(CANSendFrame));
        
        this->receive_motor_data();
    }

    void refresh_motor_status_all()
    {
        for (const auto& [name, motor] : this->motors_) 
        {
            this->refresh_motor_status(name);
        }
        return;
    }
    
    double get_position(std::string motor_name)
    {
        auto it = this->motors_.find(motor_name);
        if (it != this->motors_.end()) {
            return it->second.get_position();
        } else {
            throw std::runtime_error("Name does not exist!");
        }
    }

    double get_velocity(std::string motor_name)
    {
        auto it = this->motors_.find(motor_name);
        if (it != this->motors_.end()) {
            return it->second.get_velocity();
        } else {
            throw std::runtime_error("Name does not exist!");
        }
    }

    double get_torque(std::string motor_name)
    {
        auto it = this->motors_.find(motor_name);
        if (it != this->motors_.end()) {
            return it->second.get_torque();
        } else {
            throw std::runtime_error("Name does not exist!");
        }
    }

    int control_mit(const std::string motor_name, double kp, double kd, double q, double dq, double tau)
    {
        MotorID slave_id;
        LimitParam limit_param_cmd;

        // check and get motor from map
        auto it = this->motors_.find(motor_name);
        if (it != this->motors_.end()) {
            slave_id = it->second.get_slave_id();
            limit_param_cmd = it->second.get_limit_param();
        } else {
            fprintf(stderr, "[ ERROR ] motor_name '%s' not found!\n", motor_name.data());
            return -1;
        }

        // map linearly to given bounds
        uint16_t kp_uint    = double_to_uint(kp, 0, 500, 12);
        uint16_t kd_uint    = double_to_uint(kd, 0, 5, 12);
        uint16_t q_uint     = double_to_uint(q,   -limit_param_cmd.q,   limit_param_cmd.q,   16);
        uint16_t dq_uint    = double_to_uint(dq,  -limit_param_cmd.dq,  limit_param_cmd.dq,  12);
        uint16_t tau_uint   = double_to_uint(tau, -limit_param_cmd.tau, limit_param_cmd.tau, 12);

        // pack data
        const std::array<uint8_t, 8> data_buf{};
        data_buf[0] = (q_uint >> 8) & 0xff;
        data_buf[1] = q_uint & 0xff;
        data_buf[2] = dq_uint >> 4;
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
        data_buf[4] = kp_uint & 0xff;
        data_buf[5] = kd_uint >> 4;
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
        data_buf[7] = tau_uint & 0xff;

        // send data
        this->send_motor_data(slave_id, data_buf);

        // recveive motor response
        return this->receive_motor_data();
    }
    
    void send_motor_data(uint8_t slave_id, const std::array<uint8_t,    8>& data_buf)
    {
        CANSendFrame send_data;
        send_data.prepare(slave_id, data_buf.data());
        
        this->serial_->write((uint8_t*)&send_data, sizeof(CANSendFrame));
    }

    int receive_motor_data()
    {
        CANReceiveFrame receive_data;

        // get data from port buffer
        int rc = this->serial_->read((uint8_t*)&receive_data, 0xAA, 0x55, sizeof(CANReceiveFrame));
        if (rc < 0) {
            fprintf(stderr, "[ ERROR ] Could not receive motor status");
            return -1;
        }

        // unpack data
        switch (receive_data.cmd)
        {
        case 0x11:  // success
            return this->unpack_motor_data(&receive_data);            
        case 0x01:
            fprintf(stderr, "[ ERROR ] Receive fail.");
            return -1;
        case 0x02:
            fprintf(stderr, "[ ERROR ] Send fail.");
            return -1;
        case 0xEE:
            fprintf(stderr, "[ ERROR ] Communication Error.");
            return -1;
        case 0x08:
            fprintf(stderr, "[ ERROR ] Overvoltage.");
            return -1;
        case 0x09:
            fprintf(stderr, "[ ERROR ] Undercurrent.");
            return -1;
        case 0x0B:
            fprintf(stderr, "[ ERROR ] MOS overtemperature.");
            return -1;
        case 0x0C:
            fprintf(stderr, "[ ERROR ] motor coil overtemperature.");
            return -1;
        case 0x0D:
            fprintf(stderr, "[ ERROR ] communication loss.");
            return -1;
        case 0x0E:
            fprintf(stderr, "[ ERROR ] overload.");
            return -1;
        default:
            return -1;
        }
    }

    private:

    uint16_t double_to_uint(double x, double xmin, double xmax, uint8_t bits)
    {
        double span = xmax - xmin;
        double data_norm = std::clamp((x - xmin) / span, 0.0, 1.0);
        return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
    }

    double uint_to_double(uint16_t x, double xmin, double xmax, uint8_t bits)
    {
        double span = xmax - xmin;
        double data_norm = static_cast<double>(x) / ((1 << bits) - 1);
        double data = data_norm * span + xmin;
        return data;
    }

    int unpack_motor_data(CANReceiveFrame* receive_data)
    {
        std::string motor_name;
        
        auto it = this->lut_master_id_to_motor_name_.find(receive_data->canId);
        if (it != this->lut_master_id_to_motor_name_.end()) {
            motor_name = it->second;
        } else {
            fprintf(stderr, "[ ERROR ] Could not find motor by ID");
            return -1; // handle error
        }
        
        Motor* motor = &this->motors_.at(motor_name);
        LimitParam limit_param_receive = motor->get_limit_param();
        
        auto & data = receive_data->can_data;

        uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
        uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
        uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];
        uint8_t tmos = data[6];
        uint8_t trot = data[7];
        
        double receive_q = uint_to_double(q_uint, -limit_param_receive.q, limit_param_receive.q, 16);
        double receive_dq = uint_to_double(dq_uint, -limit_param_receive.dq, limit_param_receive.dq, 12);
        double receive_tau = uint_to_double(tau_uint, -limit_param_receive.tau, limit_param_receive.tau, 12);
        
        // update stored states
        motor->set_position(receive_q);
        motor->set_velocity(receive_dq);
        motor->set_torque(receive_tau);
        
        motor->set_tmos(tmos);
        motor->set_trot(trot);

        return 1;
    }

    void send_control_cmd(MotorID id , uint8_t cmd)
    {
        // pack data with cmd
        std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
        
        // pack CAN frame
        CANSendFrame send_data;
        send_data.prepare(id, data_buf.data());
        
        // send frame
        this->serial_->write((uint8_t*)&send_data, sizeof(CANSendFrame));
    }


    // 
    std::optional<SerialPort> serial_;
    std::unordered_map<std::string, Motor> motors_;
    std::unordered_map<MotorID, std::string> lut_master_id_to_motor_name_;
};

};

#endif