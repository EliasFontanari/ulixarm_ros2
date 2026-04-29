#include "motor.hpp"


damiao::Motor::Motor(DMMotorType motor_type, MotorID slave_id, MotorID master_id) : 
    motor_type_(motor_type), 
    slave_id_(slave_id), 
    master_id_(master_id), 
    limit_param_(limit_params[motor_type])
{
    return;
}

DMMotorType damiao::Motor::get_motor_type()
{ 
    return this->motor_type_; 
}

MotorID damiao::Motor::get_master_id()
{ 
    return this->master_id_; 
}
MotorID damiao::Motor::get_slave_id()
{ 
    return this->slave_id_; 
}

double damiao::Motor::get_position()
{ 
    return this->state_q_; 
}

void damiao::Motor::set_position(double q)
{ 
    this->state_q_ = q; 
}

double damiao::Motor::get_velocity()
{ 
    return this->state_dq_; 
}

void damiao::Motor::set_velocity(double dq) 
{ 
    this->state_dq_ = dq;
}

double damiao::Motor::get_torque()
{ 
    return this->state_tau_; 
}

void damiao::Motor::set_torque(double tau) 
{
    this->state_tau_ = tau; 
}

uint8_t damiao::Motor::get_trot()
{ 
    return this->state_trot_; 
}

void damiao::Motor::set_trot(uint8_t trot) 
{ 
    this->state_trot_ = trot; 
}

uint8_t damiao::Motor::get_tmos()
{ 
    return this->state_tmos_; 
}

void damiao::Motor::set_tmos(uint8_t tmos) 
{ 
    this->state_tmos_ = tmos; 
}

LimitParam damiao::Motor::get_limit_param()
{
    return this->limit_param_; 
}