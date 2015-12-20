#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include "ServoDriver.h"

/*  Server Controller test file
 *  Behavior: Servos with ids \in {1..12} will oscilate within 1/10 of their maximum range
 *            3 full periods of sinusoidal oscilation will occur before exit.
 *  usage:
 *     ./ServoController-Test <arduino serial port>
 */

int main(int argc, char* argv[]){
  std::vector<int> ids(3);
  ids[0] = 3;
  ids[0] = 7;
  ids[0] = 11;
  
  int baud = 115200;
  init(argv[1],baud);
  
  int N = ids.size();
  // Use torque controller
  double t = 0.0;
  while(t<5.0){
    std::vector<double> send_pos(N);
    for(int i=0;i<N;i++){
      send_pos[i] = sin( t * (M_PI*2.0)) * (M_PI/4.0) ;
    }
    
    std::cout <<send_pos<<std::endl;
    setVal(ids,send_pos);
    
    std::vector<double> pos(N), vel(N), torque(N);
    std::vector<int> recieved_ids(N);
    bool got_data = false;
    got_data = getVal(recieved_ids,pos,vel,torque);
    if(got_data){
      printf("|    POS    |    VEL    |    TOR    | \n");
      for(int i=0;i<N;i++)
        printf("|  %1.6f  |  %2.5f  |  %2.5f  |\n",pos[i],vel[i],torque[i]);
      printf("\n");
    }
    int Bps = ( baud / 10 );
    int Bytes = ( N * 3 ) ;
    double seconds_per_message = ( 1.0 / ((double) Bps) ) * ((double)Bytes);
    
    printf("Sleep %f s\n",seconds_per_message);
    
    usleep(1000 * (seconds_per_message*0.001));
    t += seconds_per_message;
  }
  
  return 0;
}
