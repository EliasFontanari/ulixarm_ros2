#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <sys/select.h>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <queue>

class SerialPort
{
    public:

    SerialPort(std::string port, speed_t baudrate, time_t timeout_ms = 2)
    {
        // open serial port
        int fd = this->open_serial_port(port);
        if (fd < 0) {
            // handle error
        }
        
        // configure sertial port
        int rc = this->configure_serial_port(fd, baudrate);
        if (!rc) {
            // hangle error
        }

        this->fd_ = fd;

        // set timeout
        this->timeout_.tv_sec = timeout_ms / 1000;
        this->timeout_.tv_usec = (timeout_ms % 1000) * 1000;

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
        if (ret == 0) {
            return 0;   // timeout
        }
        else if (ret < 0) {
            return -1;  // error
        }

        // read data
        ssize_t recv_len = ::read(this->fd_, this->recv_buf_.data(), len);

        // populate queue
        for (int i = 0; i < recv_len; i++) {
            this->recv_queue_.push(recv_buf[i]);
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

    int open_serial_port(std::string port)
    {
        int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
        return fd;
    }

    int configure_serial_port(int fd, int baudrate)
    {
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            return false;
        }

        cfsetispeed(&tty, baudrate);
        cfsetospeed(&tty, baudrate);

        tty.c_cflag
            = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
        tty.c_iflag &= ~IGNBRK; // disable break processing
        tty.c_lflag = 0; // no signaling chars, no echo, no
                        // canonical processing
        tty.c_oflag = 0; // no remapping, no delays
        tty.c_cc[VMIN] = 0; // read doesn't block
        tty.c_cc[VTIME] = 0; // non-blocking

        tty.c_iflag &= ~(IXON | IXOFF
                        | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag
            |= (CLOCAL | CREAD); // ignore modem controls,
                                // enable reading
        tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        tcflush(fd, TCIFLUSH);

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            cerr << "Error from tcsetattr: " << strerror(errno)
                << endl;
            return false;
        }
        return true;
    }


    int wait_for_data()
    {
        FD_ZERO(this->(&read_fds_));
        FD_SET(this->fd_, this->(&read_fds_));

        return select(fd_ + 1, &readFds, nullptr, nullptr, this->(&timeout_));
    }

    int fd_;
    fd_set read_fds_;
    timeval timeout_;

    std::queue<uint8_t> recv_queue_;
    std::array<uint8_t, 1024> recv_buf_;

};

#endif // SERIAL_PORT_H