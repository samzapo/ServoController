#ifndef __SERVO_DRIVER_H__
#define __SERVO_DRIVER_H__

#include <vector>
namespace ServoDriver{
  enum Parameter{
    P_POSITION  = 1,
    P_VELOCITY  = 2,
    P_LOAD      = 3,
    P_SPECIAL   = 4
  };
  
  bool init(const char* sp,std::vector<int> ids);
  
  template <class T>
  bool getVal(const std::vector<int> ids, const Parameter type, std::vector<T> val);
  
  template <class T>
  bool setVal(const std::vector<int> ids, const Parameter type, const std::vector<T> val);
}

#endif // __SERVO_DRIVER_H__