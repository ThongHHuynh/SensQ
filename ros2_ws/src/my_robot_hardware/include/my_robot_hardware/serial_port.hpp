#pragma once
#include <string>
#include <cstddef>
#include <iostream>
namespace mobile_base_hardware
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  bool open(const std::string & port, int baud);
  
  //close
  void close();

  //check whether port is open, CONST: prevent from modifying the object
  bool is_open() const;

  // Non-blocking-ish read into buffer (returns bytes read)
  int read_bytes(char * out, std::size_t max_len, int timeout_ms);

  // Write full string (returns true if all bytes written)
  bool write_string(const std::string & s);

private:
  int fd_{-1};
};

}  // namespace mobile_base_hardware
