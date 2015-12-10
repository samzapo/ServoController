#ifndef __SERVO_DRIVER_H__
#define __SERVO_DRIVER_H__

#include <vector>
#include <stdint.h>
  const uint8_t
  INST_READ       = 1   ,
  INST_WRITE      = 2   ,
  INST_REG_WRITE  = 3   ,
  INST_ACTION     = 4   ,
  INST_RESET      = 5   ,
  INST_PING       = 6   ,
  INST_SYNC_WRITE = 0x83;
  
  // INFO
  typedef uint8_t Parameter;
  const uint8_t
  TYPE_INDEX      = 0,
  PARAMETER_INDEX = 1,
  N_INDEX         = 2,
  L_INDEX         = 3,
  HEADER_SIZE     = 4;
  
  // PARAMETER
  const uint8_t
  P_EMPTY     = 0,
  P_POSITION  = 1,
  P_VELOCITY  = 2,
  P_LOAD      = 3,
  P_SPECIAL   = 4;
  
  bool init(const char* sp,std::vector<int> ids);
  
  template <class T>
  bool getVal(const std::vector<int>& ids, const Parameter type, std::vector<T>& val);
  
  template <class T>
  bool setVal(const std::vector<int>& ids, const Parameter type, const std::vector<T>& val);
  
  bool ping();

#endif // __SERVO_DRIVER_H__
