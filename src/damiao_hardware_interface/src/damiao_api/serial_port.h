#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <array>
#include <queue>

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
        return ::write(this->fd_, data, len);;
    }

    int read(uint8_t* data, uint8_t head, ssize_t len)
    {
        // wait for data to arrive, or break after timeout
        int ret = this->wait_for_data();
        if (ret == 0) return 0; // timeout
        if (ret < 0) return -1; // error

        // read data
        ssize_t recv_len = ::read(this->fd_, this->recv_buf_.data(), len);

        // populate queue
        for (int i = 0; i < recv_len; i++) {
            this->recv_queue_.push(this->recv_buf_[i]);
        }

        // search for head frame byte
        while (this->recv_queue_.size() >= len)
        {
            if(this->recv_queue_.front() != head)
            {
                this->recv_queue_.pop();
                continue;
            }
            break;
        }

        // not enough bytes for the frame
        if(this->recv_queue_.size() < len) {
            return 0;
        }

        // populate data
        for(int i = 0; i < len; i++)
        {
            data[i] = this->recv_queue_.front();
            this->recv_queue_.pop();
        }

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


    int wait_for_data()
    {
        FD_ZERO(&this->read_fds_);
        FD_SET(this->fd_, &this->read_fds_);

        // reset timeout
        timeout_.tv_sec = this->timeout_ms_ / 1000;
        timeout_.tv_usec = (this->timeout_ms_ % 1000) * 1000;

        return select(this->fd_ + 1, &this->read_fds_, nullptr, nullptr, &timeout_);
    }

    int fd_;
    fd_set read_fds_;

    time_t timeout_ms_;
    timeval timeout_;

    std::queue<uint8_t> recv_queue_;
    std::array<uint8_t, 1024> recv_buf_;

};

#endif // SERIAL_PORT_H