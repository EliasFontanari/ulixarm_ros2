#include "damiao_hardware_interface/serial_port.hpp"
#include <fcntl.h>
#include <unistd.h>

damiao::SerialPort::SerialPort()
{
    return;
}

damiao::SerialPort::~SerialPort()
{
    close(this->fd_);
    return;
}

int damiao::SerialPort::init(const char* port, speed_t baudrate, time_t timeout_ms)
{
    // open serial port
    int fd = this->open_serial_port(port);
    if (fd < 0)
        return log_error(ErrorCode::PORT_OPEN_FAILED);
    
    // configure sertial port
    int rc = this->configure_serial_port(fd, baudrate);
    if (rc < 0)
        return rc;

    this->fd_ = fd;

    // timeout
    this->timeout_ms_ = timeout_ms;

    return to_int(ErrorCode::SUCCESS);
}

int damiao::SerialPort::write(const uint8_t* data, size_t len)
{
    int rc;
    
    rc = static_cast<int>(::write(this->fd_, data, len));
    if (rc < 0)
        return log_error(ErrorCode::SEND_FAIL);
    
    return to_int(ErrorCode::SUCCESS);
}


int damiao::SerialPort::read(uint8_t* data, uint8_t head, uint8_t end, ssize_t len)
{
    // check if port still connected
    if (this->fd_ < 0)
        return log_error(ErrorCode::PORT_NOT_OPEN);
    
    int rc;
    uint8_t byte;
    ssize_t bytes_read = 0;

    // read bytes until head is found
    while (true)
    {
        rc = wait_for_byte(&byte);
        if (rc < 0)
            return rc;
        if (byte == head)
            break;
    }
    data[bytes_read] = head;
    bytes_read += 1;

    // read len - 1 remaining bytes
    while (bytes_read < len)
    {
        rc = wait_for_byte(&byte);
        if (rc < 0)
            return rc;
        data[bytes_read] = byte;
        bytes_read += 1;
    }

    // check if last byte is end
    if (data[bytes_read - 1] != end)
        return log_error(ErrorCode::WRONG_END_BYTE);
        
    return to_int(ErrorCode::SUCCESS);
}


int damiao::SerialPort::open_serial_port(const char* port)
{
    // int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    int fd = open(port, O_RDWR | O_NOCTTY);
    return fd;
}


int damiao::SerialPort::configure_serial_port(int fd, speed_t baudrate)
{
    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0)
        return log_error(ErrorCode::PORT_CONFIG_FAILED);

    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8 data bits
    tty.c_cflag &= ~PARENB;   // no parity
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;  // no hardware flow control
    tty.c_cflag |= CREAD;     // enable receiver
    tty.c_cflag |= CLOCAL;    // ignore modem control lines

    tty.c_iflag = 0;          // raw input
    tty.c_oflag = 0;          // raw output
    tty.c_lflag = 0;          // raw mode, no echo, no signals

    tty.c_cc[VMIN]  = 0;      // non-blocking read
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        return log_error(ErrorCode::PORT_CONFIG_FAILED);

    return to_int(ErrorCode::SUCCESS);
}

// int damiao::SerialPort::configure_serial_port(int fd, speed_t baudrate)
// {
//     struct termios tty{};
//     if (tcgetattr(fd, &tty) != 0)
//         return log_error(ErrorCode::PORT_CONFIG_FAILED);
    
//     cfsetispeed(&tty, baudrate);
//     cfsetospeed(&tty, baudrate);
    
//     tty.c_oflag = 0; // no remapping, no delays
//     tty.c_cflag &= ~CSIZE;
//     tty.c_cflag |= CS8; // 8
//     tty.c_cflag &= ~PARENB; // no parity
//     tty.c_cflag &= ~CSTOPB; // 1 stop bit
//     tty.c_iflag = 0;
//     tty.c_iflag &= ~INPCK; // no parity
//     tty.c_lflag = 0;
//     tty.c_lflag |= CBAUDEX; 
//     tty.c_cc[VMIN] = 0; // read doesn't block
//     tty.c_cc[VTIME] = 0; // non-blocking
    
//     tcflush(fd, TCIFLUSH);
    
//     if (tcsetattr(fd, TCSANOW, &tty) != 0)
//         return log_error(ErrorCode::PORT_CONFIG_FAILED);
    
//     return to_int(ErrorCode::SUCCESS);
// }

int damiao::SerialPort::wait_for_byte(uint8_t* out)
{
    FD_ZERO(&this->read_fds_);
    FD_SET(this->fd_, &this->read_fds_);

    this->timeout_.tv_sec = this->timeout_ms_ / 1000;
    this->timeout_.tv_usec = (this->timeout_ms_ % 1000) * 1000;

    int ret = select(this->fd_ + 1, &this->read_fds_, nullptr, nullptr, &this->timeout_);
    if (ret < 0)
        return log_error(ErrorCode::WAIT_SELECT_FAILED);
    if (ret == 0)
        return log_error(ErrorCode::WAIT_TIMEOUT);

    ssize_t n = ::read(this->fd_, out, 1);
    if (n != 1)
        return log_error(ErrorCode::READ_FAILED);
        
    return to_int(ErrorCode::SUCCESS);    
}
