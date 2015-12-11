#include <math.h>
#include <unistd.h>
#include <stdio.h>

#include "ServoDriver.h"

/*  Server Controller test file
 *  Behavior: Servos with ids \in {1..12} will oscilate within 1/10 of their maximum range
 *            3 full periods of sinusoidal oscilation will occur before exit.
 *  usage:
 *     ./ServoController-Test <arduino serial port>
 */

int main(int argc, char* argv[]){
  std::vector<int> ids(12);
  for(int i=0;i<ids.size();i++){
    ids[i] = i+1;
  }
  ids[1] = 100;
  init(argv[1],ids);
  
  typedef uint16_t ValueType;
  
  ping();
  
  double t = 0.0;
/*
  // Use position controller
  while(t<5.0){
    std::vector<ValueType> pos(ids.size());
    for(int i=0;i<ids.size();i++){
      pos[i] = (ValueType) (2*1024 + 0.1*1024.0*sin(t*2.0*M_PI));
    }
    
    getVal<ValueType>(ids,P_POSITION,pos);
    usleep(10000);
    t += 0.001;
  }
  */

  // Use torque controller
  t = 0.0;
  while(t<5.0){
    std::vector<ValueType> pos(ids.size()), vel(ids.size());
    for(int i=0;i<ids.size();i++){
      pos[i] = (ValueType) (1024 + 0.1*1024.0*sin(t*2.0*M_PI));
    }
    
//    setVal<ValueType>(ids,P_LOAD,pos);
    getVal<ValueType>(ids,P_POSITION,pos);
    getVal<ValueType>(ids,P_VELOCITY,vel);
    printf("|    POS    |    VEL    |\n");
    for(int i=0;i<ids.size();i++)
     printf("|  %7d  |  %7d  |\n",pos[i],vel[i]);
    printf("\n");
    usleep(10000);
    t += 0.001;
  }

  return 0;
}  
