#ifndef __SERVO_DRIVER_H__
#define __SERVO_DRIVER_H__

#include <vector>
#include <map>
#include <iostream>
#include <stdint.h>

  bool init(const char* sp,int baud);
  
  bool getVal(std::vector<int>& ids, std::vector<double>& pos, std::vector<double>& vel, std::vector<double>& tor);

  bool setVal(const std::vector<int>& ids, const std::vector<double>& val);
  
#endif // __SERVO_DRIVER_H__
