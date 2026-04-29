#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>

class SerialPort
{
    public:

    SerialPort(const char* port, speed_t baudrate, time_t timeout_ms = 10)
    {
        // open serial port
        int fd = this->open_serial_port(port);
        if (fd < 0) {
            fprintf(stderr, "[ ERROR ] Unable to open serial port!\n");
        } else {
            fprintf(stderr, "[ INFO ] Serial port opened!\n");
        }
        
        // configure sertial port
        int rc = this->configure_serial_port(fd, baudrate);
        if (!rc) {
            fprintf(stderr, "[ ERROR ] Unable to configure serial port!\n");
        } else {
            fprintf(stderr, "[ INFO ] Serial port configured!\n");
        }

        this->fd_ = fd;

        // timeout
        this->timeout_ms_ = timeout_ms;

        return;
    }

    ~SerialPort()
    {
        close(this->fd_);
    }

    ssize_t write(const uint8_t* data, size_t len)
    {
        return ::write(this->fd_, data, len);
    }


    int read(uint8_t* data, uint8_t head, uint8_t end, ssize_t len)
    {
        // check if port still connected
        if (this->fd_ < 0)
        {
            fprintf(stderr, "[ ERROR ] Port not connected.");
            return -1;
        }

        uint8_t byte;
        ssize_t bytes_read = 0;

        // read bytes until head is found
        while (true)
        {
            if (!wait_for_byte(&byte))
                return -1;
            if (byte == head)
                break;
        }
        data[bytes_read] = head;
        bytes_read += 1;

        // read len - 1 remaining bytes
        while (bytes_read < len)
        {
            if (!wait_for_byte(&byte))
                return -1;
            data[bytes_read] = byte;
            bytes_read += 1;
        }

        // check if last byte is end
        if (data[bytes_read - 1] != end)
            return -1;

        return 1;
    }
    
    private:

    int open_serial_port(const char* port)
    {
        // int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
        int fd = open(port, O_RDWR | O_NOCTTY);
        return fd;
    }

    bool configure_serial_port(int fd, speed_t baudrate)
    {
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(fd, &tty) != 0) {
            return false;
        }
        
        cfsetispeed(&tty, baudrate);
        cfsetospeed(&tty, baudrate);

        tty.c_oflag = 0; // no remapping, no delays
        
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8; // 8
        tty.c_cflag &= ~PARENB; // no parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        
        tty.c_iflag = 0;
        tty.c_iflag &= ~INPCK; // no parity
        
        tty.c_lflag = 0;
        tty.c_lflag |= CBAUDEX; 

        tty.c_cc[VMIN] = 0; // read doesn't block
        tty.c_cc[VTIME] = 0; // non-blocking

        tcflush(fd, TCIFLUSH);

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            return false;
        }
        return true;
    }


    bool wait_for_byte(uint8_t* out)
    {
        FD_ZERO(&this->read_fds_);
        FD_SET(this->fd_, &this->read_fds_);

        this->timeout_.tv_sec = this->timeout_ms_ / 1000;
        this->timeout_.tv_usec = (this->timeout_ms_ % 1000) * 1000;

        int ret = select(this->fd_ + 1, &this->read_fds_, nullptr, nullptr, &this->timeout_);
        if (ret <= 0)
            return false; // timeout or error

        ssize_t n = ::read(this->fd_, out, 1);
        return n == 1;
    }

    int fd_;
    fd_set read_fds_;

    time_t timeout_ms_;
    timeval timeout_;

};

#endif // SERIAL_PORT_H