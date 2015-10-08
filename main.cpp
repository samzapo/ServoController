#include <math.h>
#include <vector>
#include <unistd.h>

#include <ServoDriver.h>

int main(int argc, char* argv[]){
  double t = 0.0;
  while(t<5.0){
    std::vector<int> ids;
    std::vector<int> pos;
    for(int i=1;i<=12;i++){
      ids.push_back(1);
      pos.push_back((int) (0.1*1024.0*sin(t*2.0*M_PI)));
    }
    
    ServoDriver::setPos(ids,pos);
    usleep(1000);
    t += 0.001;
  }
}  
