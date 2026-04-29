#ifndef ULIXARM_HARDWARE__SERIAL_PORT_HPP_
#define ULIXARM_HARDWARE__SERIAL_PORT_HPP_

#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>

#inlcude "error_codes.hpp"

namespace damiao
{

    class SerialPort
    {
    public:
    
        SerialPort() = default;
        ~SerialPort() = default;
    
        int init(const char* port, speed_t baudrate, time_t timeout_ms = 10);
        int write(const uint8_t* data, size_t len);
        int read(uint8_t* data, uint8_t head, uint8_t end, ssize_t len);
    
    private:
    
        int open_serial_port(const char* port);
        int configure_serial_port(int fd, speed_t baudrate);
        int wait_for_byte(uint8_t* out);
    
        int fd_;
        fd_set read_fds_;
    
        time_t timeout_ms_;
        timeval timeout_;
    
    };

} // namespace damiao    

#endif // ULIXARM_HARDWARE__SERIAL_PORT_HPP_
