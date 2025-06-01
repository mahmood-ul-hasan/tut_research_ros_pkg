#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include "orion_rotating_base/rotating_base.h"

RotatingBase::RotatingBase() : connected_(false)
{
}

RotatingBase::~RotatingBase()
{
  if (connected_)
  {
    disconnect();
  }
}

bool RotatingBase::connect(std::string host, int port)
{
  if (!connected_)
  {
    std::cout << "Creating non-blocking socket." << std::endl;
    socket_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_fd_)
    {
      struct sockaddr_in stSockAddr;
      stSockAddr.sin_family = PF_INET;
      stSockAddr.sin_port = htons(port);
      inet_pton(AF_INET, host.c_str(), &stSockAddr.sin_addr);

      std::cout << "Connecting socket to rotating_base." << std::endl;
      std::cout << "host: " << host << ", port: " << port << std::endl;
      int ret = ::connect(socket_fd_, (struct sockaddr *) &stSockAddr, sizeof(stSockAddr));

      if (ret == 0)
      {
        connected_ = true;
        std::cout << "Connected succeeded." << std::endl;
        return true;
      }
    }
    return false;
  }
  return true;
}

void RotatingBase::disconnect()
{
  if (connected_)
  {
    close(socket_fd_);
    connected_ = false;
    std::cout << "disconnected" << std::endl;
  }
}

bool RotatingBase::isConnected() const
{
  return connected_;
}

bool RotatingBase::isRotating()
{
  char buf[13];
  sendCommand(READ, "FD", "****", buf);
  return buf[5] == '1';
}

bool RotatingBase::returnOrigin()
{
  char buf[13];
  if (!sendCommand(WRITE, "GR", "****", buf))
  {
    return false;
  }
  while (isRotating())
  {
    usleep(0.5 * 1000000);
  }
  return true;
}

bool RotatingBase::startRotationRight()
{
  char buf[13];
  return sendCommand(WRITE, "PR", "****", buf);
}

bool RotatingBase::startRotationLeft()
{
  char buf[13];
  return sendCommand(WRITE, "PL", "****", buf);
}

bool RotatingBase::stopRotation()
{
  char buf[13];
  return sendCommand(WRITE, "PE", "****", buf);
}

int RotatingBase::getSpeed()
{
  char buf[13];
  if (!sendCommand(READ, "SP", "****", buf))
  {
    return -1;
  }
  return buf[8] - '0' + 1;
}

bool RotatingBase::setSpeed(int targetSpeed)
{
  if (targetSpeed > 10)
  {
    targetSpeed = 10;
  }
  if (targetSpeed < 1)
  {
    targetSpeed = 1;
  }
  char dataStr[5] = "0000";
  dataStr[3] = targetSpeed + '0' - 1;
  char buf[13];
  return sendCommand(WRITE, "SP", dataStr, buf);
}

double RotatingBase::getPosition()
{
  char buf[13];
  if (!sendCommand(READ, "PD", "****", buf))
  {
    return 361.0;
  }
  char dataStr[5] = "0000";
  for (size_t i = 0; i < 4; i++)
  {
    dataStr[i] = buf[i + 5];
  }
  int returnPos = strtol(dataStr, NULL, 16);
  if (returnPos > 0x8000)
  {
    returnPos = returnPos - 0xffff - 1;
  }
  return static_cast<double>(returnPos) / 10;
}

double RotatingBase::getTargetPos()
{
  char buf[13];
  if (!sendCommand(READ, "PV", "****", buf))
  {
    return 361.0;
  }
  char dataStr[5] = "0000";
  for (size_t i = 0; i < 4; i++)
  {
    dataStr[i] = buf[i + 5];
  }
  int returnPos = strtol(dataStr, NULL, 16);
  if (returnPos > 0x8000)
  {
    returnPos = returnPos - 0xffff - 1;
  }
  return static_cast<double>(returnPos) / 10;
}

bool RotatingBase::setTargetPos(double targetPos)
{
  // 上限下限値を調べておく
  if (targetPos > 360.0)
  {
    targetPos = 360.0;
  }
  if (targetPos < -360.0)
  {
    targetPos = -360.0;
  }
  targetPos *= 10;
  int intPos = static_cast<int>(std::round(targetPos));
  if (intPos < 0)
  {
    intPos = intPos + 0xffff + 1;
  }
  char dataStr[5];
  hex2str(intPos, 4, dataStr);
  char buf[13];
  return sendCommand(WRITE, "PV", dataStr, buf);
}

bool RotatingBase::goTargetPos()
{
  char buf[13];
  if (!sendCommand(WRITE, "GO", "****", buf))
  {
    return false;
  }
  while (isRotating())
  {
    usleep(0.5 * 1000000);
  }
  return true;
}

bool RotatingBase::sendCommand(enum READorWRITE RorW, const char* cmdStr, const char* dataStr, char* buf)
{
  char bf1[9];
  char RorWChar;
  if (RorW == READ)
  {
    RorWChar = 'R';
  } else {
    RorWChar = 'W';
  }
  sprintf(bf1, "%c%s%s%s", RorWChar, "00", cmdStr, dataStr);

  // calculate BCC code
  char tmpBf = bf1[0] ^ bf1[1];
  for (size_t i = 2; i < 9; i++)
  {
    tmpBf = tmpBf ^ bf1[i];
  }
  char bccBf[3];
  hex2str((int)tmpBf, 2, bccBf);

  char writeBf[12];
  sprintf(writeBf, "%s%s%c", bf1, bccBf, 0x0D);

  mutex_.lock();
  write(socket_fd_, writeBf, strlen(writeBf));
  int len = read(socket_fd_, buf, 100);
  mutex_.unlock();

  buf[len] = 0;
  if (buf[0] == 0x15)
  {
    std::cerr << "error response recieved. code: " << buf[1] << std::endl;
    return false;
  }
  if (buf[0] != RorWChar)
  {
    std::cerr << "invalid response recieved." << std::endl;
    std::cerr << "command: " << writeBf << std::endl;
    std::cerr << "response: " << buf << std::endl;
    return false;
  }
  return true;
}

inline int RotatingBase::hex2str(int inputHex, int nOfDigits, char* outputStr) const
{
  return sprintf(outputStr, "%0*X", nOfDigits, inputHex);
}
