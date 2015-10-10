#ifndef __SERVO_DRIVER_H__
#define __SERVO_DRIVER_H__

#include <vector>
namespace ServoDriver{
  bool init(const char* sp,std::vector<int> ids);
  
  bool getPos(const std::vector<int> ids, std::vector<int> val);
  bool setPos(const std::vector<int> ids, const std::vector<int> pos);
  
  bool getVel(const std::vector<int> ids, std::vector<int> val);
  bool setVel(const std::vector<int> ids, std::vector<int> val);
  
  bool getLoad(const std::vector<int> ids, std::vector<int> val);
  bool setLoad(const std::vector<int> ids, std::vector<int> val);
}

#endif // __SERVO_DRIVER_H__