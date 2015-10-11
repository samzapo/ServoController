#include <math.h>
#include <unistd.h>

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
  ServoDriver::init(argv[1],ids);
  double t = 0.0;
  while(t<5.0){
    std::vector<int> pos(ids.size());
    for(int i=0;i<ids.size();i++){
      pos[i] = (int) (2*1024 + 0.1*1024.0*sin(t*2.0*M_PI));
    }
    
    ServoDriver::setVal<int>(ids,ServoDriver::P_POSITION,pos);
    usleep(1000);
    t += 0.001;
  }

  return 0;
}  
