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
  std::cout << "isRotating(): " << buf << std::endl;
  std::cout << "isRotating(): " << buf[0] << std::endl;
  return buf[0] == '1';
}

/*
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

*/

bool RotatingBase::returnOrigin()
{
  double curent_pos;
  double target_pos_;
  target_pos_ = 90;
  //mutex_.lock();
  curent_pos = getPosition();
  //mutex_.unlock();

  std::cout << "curent_pos: " << curent_pos << std::endl;
  std::cout << "target_pos_: " << target_pos_ << std::endl;
  if (curent_pos != target_pos_)
  {    
    
      setTargetPos(target_pos_);
      goTargetPos();

      while (isRotating())
      {
        usleep(0.5 * 1000);
      }
 }

    if (curent_pos >= target_pos_ -1  || curent_pos <= target_pos_ +1)
    {
        std::cout << "returnOrigin() sucessed : " << std::endl;
        std::cout << "curent_pos: " << curent_pos << std::endl;
        std::cout << "target_pos_: " << target_pos_ << std::endl;
        return true;}
    else {
        std::cout << "returnOrigin() failed : " << std::endl;
        std::cout << "curent_pos: " << curent_pos << std::endl;
        std::cout << "target_pos_: " << target_pos_ << std::endl;
        return false; }


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
  return sendCommand(WRITE, "ST", "****", buf);
}

int RotatingBase::getSpeed()
{
  char buf[13];
  //if (!sendCommand(READ, "SP", "000X", buf))
  if (!sendCommand(READ, "SP", "****", buf))
  {
    std::cout << "RotatingBase::getSpeed() failed : " << std::endl;
    std::cout << "returnSpeed: " << buf[8] - '0' + 1 << std::endl;
    return -1;
  }
  //std::cout << "returnSpeed: " << buf[8] - '0' + 1 << std::endl;
  return buf[8] - '0' + 1;
}

bool RotatingBase::setSpeed(int targetSpeed)
{
  if (targetSpeed > 3)
  {
    targetSpeed = 3;
  }
  if (targetSpeed < 0)
  {
    targetSpeed = 0;
  }
  //char dataStr[5] = "000X";
  char dataStr[5] = "****";
  dataStr[3] = targetSpeed + '0' - 1;
  char buf[13];
  return sendCommand(WRITE, "SP", dataStr, buf);
}

double RotatingBase::getPosition()
{
  char buf[13];
  //if (!sendCommand(READ, "PD", "XXXX", buf))
  if (!sendCommand(READ, "PD", "****", buf))
  {
    std::cout << "RotatingBase::getPosition() failed : " << std::endl;
    return 361.0;

  }
 // char dataStr[5] = "XXXX";
  char dataStr[5] = "****";
  for (size_t i = 0; i < 4; i++)
  {
    dataStr[i] = buf[i + 5];
  }
  int returnPos = strtol(dataStr, NULL, 16);
  
  //std::cout << "dataStr: " << dataStr << std::endl;

  //std::cout << "returnPos: " << returnPos << std::endl;

  if (returnPos > 0x8000)
  {
    returnPos = returnPos - 0xffff - 1;
  }
  //std::cout << "returnPos: " << returnPos/100<< std::endl;
  
  if (!sendCommand(READ, "PD", "****", buf))
  {
    std::cout << "RotatingBase::getPosition() failed : " << std::endl;
    std::cout << "returnPos: " << returnPos/100<< std::endl;
    return 361.0;

  }

  return static_cast<double>(returnPos) / 100;

}

double RotatingBase::getTargetPos()
{
  char buf[13];
  if (!sendCommand(READ, "PV", "****", buf))
  {
    std::cout << "RotatingBase::getTargetPos() failed : " << std::endl;
    return 361.0;
  }
  //char dataStr[5] = "XXXX";
  char dataStr[5] = "****";
  for (size_t i = 0; i < 4; i++)
  {
    dataStr[i] = buf[i + 5];
  }
  int returnPos = strtol(dataStr, NULL, 16);
  if (returnPos > 0x8000)
  {
    returnPos = returnPos - 0xffff - 1;
  }
  return static_cast<double>(returnPos) / 100;
}

bool RotatingBase::setTargetPos(double targetPos)
{
  // 上限下限値を調べておく
  if (targetPos > 270.0)
  {
    targetPos = 270.0;
  }
  if (targetPos < 90.0)
  {
    targetPos = 90.0;
  }
  targetPos *= 100;
  int intPos = static_cast<int>(std::round(targetPos));
  if (intPos < 0)
  {
    intPos = intPos + 0xffff + 1;
  }
  char dataStr[5];
  std::cout << "intPos__setTargetPos : " << intPos << std::endl;
  hex2str(intPos, 4, dataStr);
  char buf[13];
  //return sendCommand(WRITE, "PV", dataStr, buf);

  if (!sendCommand(WRITE, "PV", dataStr, buf))
  {
    std::cout << "setTargertPos() failed : " << std::endl;
    return false;
  }

  if (sendCommand(WRITE, "PV", dataStr, buf))
  {
    std::cout << "setTargertPos() successed : " << std::endl;
    return true;
  }


}

bool RotatingBase::goTargetPos()
{
  char buf[13];
  if (!sendCommand(WRITE, "GO", "****", buf))
  {
    std::cout << "goTargetPos() failed : " << std::endl;
    return false;
  }
  while (isRotating())
  {
    usleep(0.5 * 1000000);
  }
  std::cout << "goTargetPos() successed : " << std::endl;
  return true;
}




bool RotatingBase::sendCommand(enum READorWRITE RorW, const char* cmdStr, const char* dataStr, char* buf)
{
  char bf1[9];
  char RorWChar;
  time_t pervious_time = time(nullptr);
  time_t current_time;

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
    std::cerr << "command failed  : " << writeBf << std::endl;
    std::cerr << "response failed : " << buf << std::endl;
    std::cerr << "buf[0]: " << buf[0] << std::endl;
    std::cerr << "buf[1]: " << buf[1] << std::endl;
    std::cerr << "buf[2]: " << buf[2] << std::endl;
    return false;
  }

  /*
  current_time = time(nullptr);
  if (current_time >  pervious_time +  (0.8)){
  if (buf[0] == RorWChar)
  {
    std::cerr << "valid response recieved." << std::endl;
    std::cerr << "command sucessed: " << writeBf << std::endl;
    std::cerr << "response sucessed: " << buf << std::endl;
    std::cerr << "buf[0]: " << buf[0] << std::endl;
    std::cerr << "buf[1]: " << buf[1] << std::endl;
    std::cerr << "buf[2]: " << buf[2] << std::endl;
  }
  }
  pervious_time = current_time ;
   */
   
 /*

    valid response recieved.
    command sucessed:  R00PDXXXX46
    response sucessed: R00PD465041
    buf[0]: R
    buf[1]: 0
    buf[2]: 0

    valid response recieved.
    command sucessed: R00SP000X39
    response sucessed: R00SP000253
    buf[0]: R
    buf[1]: 0
    buf[2]: 0
 
 */

  return true;
}

inline int RotatingBase::hex2str(int inputHex, int nOfDigits, char* outputStr) const
{
  return sprintf(outputStr, "%0*X", nOfDigits, inputHex);
}
