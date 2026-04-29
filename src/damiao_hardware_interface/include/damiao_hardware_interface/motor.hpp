#ifndef ULIXARM_HARDWARE__MOTOR_HPP_
#define ULIXARM_HARDWARE__MOTOR_HPP_

#include <cstdint>

namespace damiao
{

    using MotorID = uint32_t;
    
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
    
    extern LimitParam limit_params[NUM_OF_MOTORS];
    
    class Motor
    {
    public:
    
        Motor(DMMotorType motor_type, MotorID slave_id, MotorID master_id);
        Motor() = default;
    
        DMMotorType get_motor_type();
    
        MotorID get_master_id();
        MotorID get_slave_id();
    
        double get_position();
        void set_position(double q);
        
        double get_velocity();
        void set_velocity(double dq);
        
        double get_torque();
        void set_torque(double tau);
    
        uint8_t get_trot();
        void set_trot(uint8_t trot);
        
        uint8_t get_tmos();
        void set_tmos(uint8_t tmos);
        
        LimitParam get_limit_param();
    
    private:
        DMMotorType motor_type_;
        MotorID slave_id_;
        MotorID master_id_;
    
        LimitParam limit_param_{};
    
        double state_q_     = 0.0;
        double state_dq_    = 0.0;
        double state_tau_   = 0.0;
        uint8_t state_trot_ = 0;
        uint8_t state_tmos_ = 0;
    };

} // namespace damiao    


#endif // ULIXARM_HARDWARE__MOTOR_HPP_