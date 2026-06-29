// Copyright 2021 ros2_control Development Team
// Adapted from github.com/konu-droid/diffdrive_ros2_control (Apache-2.0).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef VOLCANIBOT_HARDWARE_INTERFACE__ROBOTEQ_COMMS_HPP_
#define VOLCANIBOT_HARDWARE_INTERFACE__ROBOTEQ_COMMS_HPP_

#include <iostream>
#include <sstream>
#include <string>

#include <libserial/SerialPort.h>

namespace volcanibot_hardware_interface
{

inline LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 9600:   return LibSerial::BaudRate::BAUD_9600;
    case 19200:  return LibSerial::BaudRate::BAUD_19200;
    case 38400:  return LibSerial::BaudRate::BAUD_38400;
    case 57600:  return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cerr << "Unsupported baud rate " << baud_rate << ", defaulting to 115200" << std::endl;
      return LibSerial::BaudRate::BAUD_115200;
  }
}

class RoboteqComm
{
public:
  bool connect(const std::string & port, int baud_rate, int timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    try
    {
      serial_.Open(port);
      serial_.SetBaudRate(convert_baud_rate(baud_rate));
      send_msg("^ECHOF 1\r");  // disable controller echo
    }
    catch (const std::exception & e)
    {
      std::cerr << "Roboteq serial open failed: " << e.what() << std::endl;
      return false;
    }
    return serial_.IsOpen();
  }

  bool disconnect()
  {
    if (serial_.IsOpen()) serial_.Close();
    return true;
  }

  bool connected() { return serial_.IsOpen(); }

  std::string send_msg(const std::string & msg)
  {
    serial_.FlushIOBuffers();
    serial_.Write(msg);
    serial_.DrainWriteBuffer();

    std::string response;
    try
    {
      serial_.ReadLine(response, '\r', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      // Caller decides whether a timeout is fatal.
    }
    return response;
  }

  // Send velocity commands in motor RPM. Channel 1 = left, channel 2 = right.
  void drive(double left_rpm, double right_rpm)
  {
    std::stringstream ls, rs;
    ls << "!S 1 " << static_cast<int>(left_rpm) << "\r";
    rs << "!S 2 " << static_cast<int>(right_rpm) << "\r";
    send_msg(ls.str());
    send_msg(rs.str());
  }

  // Read motor RPM via Roboteq's ?S query. Response format: "S=<ch1>:<ch2>\r".
  // Convention here matches DriveCommand: channel 1 -> left, channel 2 -> right.
  bool read_rpm(double & left_rpm, double & right_rpm)
  {
    const std::string response = send_msg("?S\r");
    if (response.empty()) return false;

    int ch1 = 0, ch2 = 0;
    if (std::sscanf(response.c_str(), "S=%d:%d", &ch1, &ch2) != 2) return false;
    left_rpm = static_cast<double>(ch1);
    right_rpm = static_cast<double>(ch2);
    return true;
  }

  // Read absolute encoder counters via Roboteq's ?C query. Response format:
  // "C=<ch1>:<ch2>\r", counts are signed 32-bit. Channel 1 -> left, 2 -> right.
  bool read_counts(long & left_count, long & right_count)
  {
    const std::string response = send_msg("?C\r");
    if (response.empty()) return false;

    long ch1 = 0, ch2 = 0;
    if (std::sscanf(response.c_str(), "C=%ld:%ld", &ch1, &ch2) != 2) return false;
    left_count = ch1;
    right_count = ch2;
    return true;
  }

private:
  LibSerial::SerialPort serial_;
  int timeout_ms_ = 1000;
};

}  // namespace volcanibot_hardware_interface

#endif  // VOLCANIBOT_HARDWARE_INTERFACE__ROBOTEQ_COMMS_HPP_
