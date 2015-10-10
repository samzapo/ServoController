#include <math.h>
#include <unistd.h>

#include "ServoDriver.h"

/*  Server Controller test file
 *  Bahavior: Servos with ids \in {1..12} will oscilate within 1/10 of their maximum range
 *            3 full periods of sinusoidal oscilation will occur before exit.
 */

int main(int argc, char* argv[]){
  std::vector<int> ids;
  ServoDriver::init(argv[1],ids);
  double t = 0.0;
  while(t<5.0){
    std::vector<int> pos;
    for(int i=1;i<=ids.size();i++){
      pos.push_back((int) (0.1*1024.0*sin(t*2.0*M_PI)));
    }
    
    ServoDriver::setPos(ids,pos);
    usleep(1000);
    t += 0.001;
  }

  return 0;
}  
