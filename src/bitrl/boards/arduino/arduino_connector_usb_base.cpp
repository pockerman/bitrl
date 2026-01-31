//
// Created by alex on 8/2/25.
//
#include "bitrl/boards/arduino/arduino_connector_usb_base.h"

#include <chrono>
#include <cstring> // memset
#include <fcntl.h> // open
#include <iostream>
#include <string>
#include <termios.h> // POSIX terminal control
#include <thread>
#include <unistd.h> // read/write/close

namespace bitrl
{
namespace boards::arduino
{
ArduinoConnectorUSBBase::ArduinoConnectorUSBBase(const std::string &port)
    : ArduinoConnectorBase(), port_name_(port)
{
}

void ArduinoConnectorUSBBase::connect()
{
    open_port_();
    configure_port_();
}

void ArduinoConnectorUSBBase::close_connection() { close(this->port_id()); }

void ArduinoConnectorUSBBase::open_port_()
{
    int port_id;
    if (port_id = open(port_name_.c_str(), O_RDWR | O_NOCTTY); port_id < 0)
    {
        std::cerr << "Failed to open port " << port_name_ << std::endl;
        return;
    }

    this->set_port_id_(port_id);
}

void ArduinoConnectorUSBBase::configure_port_()
{
    if (!this->is_port_open())
    {
    }

    termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(this->port_id(), &tty) != 0)
    {
        std::cerr << "Error getting termios attributes" << std::endl;
        return;
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8-bit characters
    tty.c_cflag &= ~PARENB;  // No parity
    tty.c_cflag &= ~CSTOPB;  // One stop bit
    tty.c_cflag &= ~CRTSCTS; // No flow control

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // No flow control
    tty.c_oflag &= ~OPOST;                          // Raw output

    tcsetattr(this->port_id(), TCSANOW, &tty);

    // Give the Arduino time to reset
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

std::string ArduinoConnectorUSBBase::send_cmd(const ArduinoCMDBase &cmd)
{
    if (is_port_open())
    {
        std::cerr << "Port " << port_name_ << " is not open" << std::endl;
        return "";
    }

    std::string to_send = cmd.get_cmd() + "\n";
    write(this->port_id(), to_send.c_str(), to_send.size());

    std::cout << "Sent: " << to_send << std::endl;

    char buffer[256];
    int n = read(this->port_id(), buffer, sizeof(buffer));
    if (n > 0)
    {
        std::string response(buffer, n);
        // Remove trailing newline and carriage return
        response.erase(response.find_last_not_of("\r\n") + 1);
        return response;
    }
    return "No response from Arduino.";
}

} // namespace boards::example_15
} // namespace bitrl
