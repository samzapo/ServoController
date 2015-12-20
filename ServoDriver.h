#ifndef __SERVO_DRIVER_H__
#define __SERVO_DRIVER_H__

#include <vector>
#include <map>
#include <iostream>
#include <stdint.h>

template < class T >
std::ostream& operator << (std::ostream& os, const std::vector<T>& v)
{
  os << "("<< v.size() <<")[";
  for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
  {
    os << " " << *ii;
  }
  os << "]";
  return os;
}

template < class T >
std::ostream& operator << (std::ostream& os, const std::map<std::string, T >& v)
{
  os << "("<< v.size() <<")[" << std::endl;
  for (typename std::map<std::string, T >::const_iterator ii = v.begin(); ii != v.end(); ++ii)
  {
    os << " " << ii->first << " = " << ii->second << std::endl;
  }
  os << "]";
  return os;
}

  bool init(const char* sp,int baud);
  
  bool getVal(std::vector<int>& ids, std::vector<double>& pos, std::vector<double>& vel, std::vector<double>& tor);

  bool setVal(const std::vector<int>& ids, const std::vector<double>& val);
  
#endif // __SERVO_DRIVER_H__
