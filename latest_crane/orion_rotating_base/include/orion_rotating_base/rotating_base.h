#ifndef ROTATINGBASE_H_
#define ROTATINGBASE_H_

#include <string>
#include <mutex>

class RotatingBase
{
public:
  const std::string DEFAULT_HOST = "192.168.0.20";  
  /* const std::string DEFAULT_HOST = "192.168.10.50"; */
  static constexpr int DEFAULT_PORT = 61055;
  static constexpr double DEFAULT_FREQUENCY = 16.0;
  static constexpr int MAX_SPEED = 3;

  RotatingBase();
  virtual ~RotatingBase();

  bool connect(std::string host, int port);
  void disconnect();
  bool isConnected() const;
  bool isRotating();
  bool returnOrigin();
  bool startRotationRight();
  bool startRotationLeft();
  bool stopRotation();
  int getSpeed();
  bool setSpeed(int targetSpeed);
  double getPosition();
  double getTargetPos();
  bool setTargetPos(double targetPos);
  bool goTargetPos();

protected:
  bool connected_;
  int socket_fd_;
  std::mutex mutex_;
  enum READorWRITE
  {
    READ,
    WRITE
  };
  
  bool sendCommand(enum READorWRITE RorW, const char* cmdStr, const char* dataStr, char* buf);
  bool sendSpeedCommand(enum READorWRITE RorW, const char* cmdStr, const char* dataStr, char* buf);
  inline int hex2str(int inputHex, int nOfDigits, char* outputStr) const;
};

#endif /* ROTATINGBASE_H_ */
