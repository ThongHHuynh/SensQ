#include "my_robot_hardware/serial_port.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <cstring>

namespace mobile_base_hardware
{
//speed_t: termios.h
static speed_t baud_to_speed(int baud) //return baudrate
{
  switch (baud) 
  {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

SerialPort::~SerialPort() { close(); std::cout<<"Port Closed."; }

bool SerialPort::open(const std::string & port, int baud)
{
  close();

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); //global namespace for open
  if (fd_ < 0) return false;

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) // copy current serial configuration in linux to tty
  {
    close();
    return false;
  }

  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD); //enable receiver
  tty.c_cflag &= ~CSTOPB;          //stop 1 bit
  tty.c_cflag &= ~CRTSCTS;         
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  const speed_t spd = baud_to_speed(baud); //store baudrate in spd
  cfsetispeed(&tty, spd); //set input, output speed
  cfsetospeed(&tty, spd);

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) //take modified tty setting apply to fd_ handler
  {
    close();
    return false;
  }

  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
    std::cout<<"Port closed";
  }
}

bool SerialPort::is_open() const { return fd_ >= 0; }

int SerialPort::read_bytes(char * out, std::size_t max_len, int timeout_ms)
{
  if (fd_ < 0) return -1; //check if port is open

  fd_set set; //monitor fd
  FD_ZERO(&set); //clear the set
  FD_SET(fd_, &set); //add fd_ to the set

  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int rv = select(fd_ + 1, &set, nullptr, nullptr, &tv); //handle timeout, wait for set data with tv time
  if (rv <= 0) return 0;

  int n = ::read(fd_, out, max_len); //read from fd_, store to out, with up to max_len value
  if (n < 0) return 0;
  return n;
}

bool SerialPort::write_string(const std::string & s)
{
  if (fd_ < 0) return false; //check port

  const char * p = s.c_str();
  std::size_t left = s.size(); //number of bytes to fill 

  while (left > 0)
  {
    int n = ::write(fd_, p, left); //write left number of bytes, starting from p
    if (n < 0) return false;
    left -= static_cast<std::size_t>(n); //update left
    p += n;
  }
  return true;
}

}  // namespace my_robot_hardware
