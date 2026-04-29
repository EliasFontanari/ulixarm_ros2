#include "motor_control.hpp"

int damiao::MotorControl::init(const char* port, speed_t baudrate)
{
    int rc;
    rc = this->serial_.init(port, baudrate, 10);
    if (rc < 0)
        return rc;
    
    return to_int(ErrorCode::SUCCESS);
}

void damiao::MotorControl::add_motor(const std::string motor_name, const DMMotorType motor_type, 
    const MotorID slave_id, const MotorID master_id)
{
    Motor motor(motor_type, slave_id, master_id);
    this->motors_[motor_name] = motor;
    this->lut_master_id_to_motor_name_[master_id] = motor_name;
    return;
}

int damiao::MotorControl::enable_motor(const std::string motor_name)
{
    auto it = this->motors_.find(motor_name);
    if (it != this->motors_.end()) {
        MotorID slave_id = it->second.get_slave_id();
        this->send_control_cmd(slave_id, 0xFC);
    } else {
        return log_error(ErrorCode::MOTOR_NOT_FOUND);
    }

    return to_int(ErrorCode::SUCCESS);
}

int damiao::MotorControl::enable_motor_all()
{
    int rc;

    for (const auto& [name, motor] : this->motors_) 
    {
        rc = this->enable_motor(name);
        if (rc < 0)
            return rc;
    }

    return rc;
}

int damiao::MotorControl::disable_motor(const std::string motor_name)
{
    int rc;

    auto it = this->motors_.find(motor_name);
    if (it != this->motors_.end()) {
        MotorID slave_id = it->second.get_slave_id();
        rc = this->send_control_cmd(slave_id, 0xFD);
    } else {
        return log_error(ErrorCode::MOTOR_NOT_FOUND);
    }

    return rc;
}

int damiao::MotorControl::disable_motor_all()
{
    int rc;

    for (const auto& [name, motor] : this->motors_) 
    {
        rc = this->disable_motor(name);
        if (rc < 0) 
            return rc;
    }

    return rc;
}

int damiao::MotorControl::refresh_motor_status(const std::string motor_name)
{
    int rc;
    MotorID slave_id;

    auto it = this->motors_.find(motor_name);
    if (it != this->motors_.end()) {
        slave_id = it->second.get_slave_id();
    } else {
        return log_error(ErrorCode::MOTOR_NOT_FOUND);
    }
    
    uint32_t id = 0x7FF;
    
    uint8_t can_low = slave_id & 0xff; // id low 8 bit
    uint8_t can_high = (slave_id >> 8) & 0xff; //id high 8 bit
    std::array<uint8_t, 8> data_buf = {can_low,can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    CANSendFrame send_data;
    send_data.prepare(id, data_buf.data());
    rc = this->serial_.write((uint8_t*)&send_data, sizeof(CANSendFrame));
    if (rc < 0)
        return log_error(ErrorCode::SEND_FAIL);

    rc = this->receive_motor_data();
    if (rc < 0)
        return rc;

    return to_int(ErrorCode::SUCCESS);
}

int damiao::MotorControl::refresh_motor_status_all()
{
    int rc;
    
    for (const auto& [name, motor] : this->motors_) 
    {
        rc = this->refresh_motor_status(name);
        if (rc < 0)
            return rc;
    }

    return rc;
}

int damiao::MotorControl::get_position(std::string motor_name, double& position)
{
    auto it = this->motors_.find(motor_name);
    if (it != this->motors_.end()) {
        position = it->second.get_position();
    } else {
        return log_error(ErrorCode::MOTOR_NOT_FOUND);
    }

    return to_int(ErrorCode::SUCCESS);
}

int damiao::MotorControl::get_velocity(std::string motor_name, double& velocity)
{
    auto it = this->motors_.find(motor_name);
    if (it != this->motors_.end()) {
        velocity = it->second.get_velocity();
    } else {
        return log_error(ErrorCode::MOTOR_NOT_FOUND);
    }

    return to_int(ErrorCode::SUCCESS);
}

int damiao::MotorControl::get_torque(std::string motor_name, double& torque)
{
    auto it = this->motors_.find(motor_name);
    if (it != this->motors_.end()) {
        torque = it->second.get_torque();
    } else {
        return log_error(ErrorCode::MOTOR_NOT_FOUND);
    }

    return to_int(ErrorCode::SUCCESS);
}

int damiao::MotorControl::control_mit(const std::string motor_name, double kp, double kd, double q, double dq, double tau)
{
    int rc;
    MotorID slave_id;
    LimitParam limit_param_cmd;

    // check and get motor from map
    auto it = this->motors_.find(motor_name);
    if (it != this->motors_.end()) {
        slave_id = it->second.get_slave_id();
        limit_param_cmd = it->second.get_limit_param();
    } else {
        return log_error(ErrorCode::MOTOR_NOT_FOUND);
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

    // send data
    rc = this->send_motor_data(slave_id, data_buf);
    if (rc < 0)
        return rc;

    // recveive motor response
    rc = this->receive_motor_data();
    if (rc < 0)
        return rc;

    return to_int(ErrorCode::SUCCESS);
}

int damiao::MotorControl::send_motor_data(uint8_t slave_id, const std::array<uint8_t,8>& data_buf)
{
    int rc;
    CANSendFrame send_data;
    send_data.prepare(slave_id, data_buf.data());
    
    rc = this->serial_.write((uint8_t*)&send_data, sizeof(CANSendFrame));
    if (rc < 0)
        return log_error(ErrorCode::SEND_FAIL);

    return to_int(ErrorCode::SUCCESS);
}

int damiao::MotorControl::receive_motor_data()
{
    int rc;
    CANReceiveFrame receive_data;

    // get data from port buffer
    rc = this->serial_.read((uint8_t*)&receive_data, 0xAA, 0x55, sizeof(CANReceiveFrame));
    if (rc < 0)
        return rc;

    // unpack data
    switch (receive_data.cmd)
    {
    case 0x11:  // success
        rc = this->unpack_motor_data(&receive_data);
        if (rc < 0)
            return rc;
        break;

    case 0x01:
        log_error(ErrorCode::RECEIVE_FAIL);
    case 0x02:
        log_error(ErrorCode::SEND_FAIL);
    case 0xEE:
        log_error(ErrorCode::COMMUNICATION);
    case 0x08:
        log_error(ErrorCode::OVERVOLTAGE);
    case 0x09:
        log_error(ErrorCode::UNDERVOLTAGE);
    case 0x0B:
        log_error(ErrorCode::MOS_OVERTEMPERATURE);
    case 0x0C:
        log_error(ErrorCode::COIL_OVERTEMPERATURE);
    case 0x0D:
        log_error(ErrorCode::COMMUNICATION_LOSS);
    case 0x0E:
        log_error(ErrorCode::OVERLOAD);
    default:
        log_error(ErrorCode::UNKNOWN_CMD);
    }       

    return to_int(ErrorCode::SUCCESS);
}





uint16_t damiao::MotorControl::double_to_uint(double x, double xmin, double xmax, uint8_t bits)
{
    double span = xmax - xmin;
    double data_norm = std::clamp((x - xmin) / span, 0.0, 1.0);
    return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
}

double damiao::MotorControl::uint_to_double(uint16_t x, double xmin, double xmax, uint8_t bits)
{
    double span = xmax - xmin;
    double data_norm = static_cast<double>(x) / ((1 << bits) - 1);
    double data = data_norm * span + xmin;
    return data;
}

int damiao::MotorControl::unpack_motor_data(CANReceiveFrame* receive_data)
{
    int rc;
    std::string motor_name;
    
    auto it = this->lut_master_id_to_motor_name_.find(receive_data->can_id);
    if (it != this->lut_master_id_to_motor_name_.end()) {
        motor_name = it->second;
    } else {
        return log_error(ErrorCode::MOTOR_NOT_FOUND);
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

    return to_int(ErrorCode::SUCCESS);
}

int damiao::MotorControl::send_control_cmd(MotorID id , uint8_t cmd)
{
    int rc;

    // pack data with cmd
    std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
    
    // pack CAN frame
    CANSendFrame send_data;
    send_data.prepare(id, data_buf.data());
    
    // send frame
    rc = this->serial_.write((uint8_t*)&send_data, sizeof(CANSendFrame));
    if (rc < 0)
        return log_error(ErrorCode::SEND_FAIL);

    return to_int(ErrorCode::SUCCESS);        
}
