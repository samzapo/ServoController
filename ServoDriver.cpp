#include "ServoDriver.h"

#include <iostream>
#include <stdio.h>    // Standard input/output definitions
#include <stdlib.h>
#include <string.h>   // String function definitions
#include <unistd.h>   // for usleep()
#include <assert.h>
#include <sstream>
#include <math.h>


#define println(str) std::cerr << str<< std::endl
#define print(str)   std::cerr << str

void error(char* msg)
{
  fprintf(stderr, "%s\n",msg);
//  exit(EXIT_FAILURE);
}

const int buf_max = 0xFF;

int fd = -1;
uint8_t quiet=1;
int rc,n;


/////////////////////////////////////////////////////////////////////////
///////////////////////////  Serial Handling ////////////////////////////

extern "C" {
#include "arduino-serial-lib.h"
}

/// Fills 'ids' with ids of servos that are accessible
bool init(const char* sp,int baudrate){
  
  if( fd!=-1 ) {
    serialport_close(fd);
    if(!quiet) fprintf(stdout, "closed port %s\n",sp);
  }
  fd = serialport_init(sp, baudrate);
  if( fd==-1 ) error("couldn't open port");
  if(!quiet) fprintf(stdout, "opened port %s\n",sp);
  
  serialport_flush(fd);
  
  return true;
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////  Control Handling ////////////////////////////
#define VAL_FFFF 65535.0
#define VAL_FFFF_2 VAL_FFFF/2.0
const double MAX_POSITION = M_PI;
const double MAX_SPEED  =  20.0;
const double MAX_TORQUE =  20.0;

double uint16_to_real(int uint16_value, double max_value){
  return (( uint16_value - VAL_FFFF_2) / VAL_FFFF_2) * max_value;
}

int real_to_uint16(double real_value, double max_value){
  int return_val = ((real_value/max_value) * VAL_FFFF_2) + VAL_FFFF_2;
  return return_val - (return_val % 2);
}

int concat_uint8_to_uint16(uint8_t L_BYTE, uint8_t H_BYTE){
  return (L_BYTE % 0x100) + (((H_BYTE % 0x100 ) << 8) & 0xFF00);
}

void break_uint16_to_uint8(int uint16_value,uint8_t * L_BYTE, uint8_t * H_BYTE){
  *L_BYTE = (uint8_t) (uint16_value % 0x100);
  *H_BYTE = (uint8_t) (uint16_value / 0x100);
}

static int HEADER_SIZE = 2;

bool setVal(const std::vector<int>& ids, const std::vector<double>& val){
  if( fd == -1 ) error("serial port not opened");
  int N = ids.size();
  uint8_t buf[0xFF];
  int index = 0;
  for(int i=0;i<HEADER_SIZE;i++){
    serialport_writebyte(fd,0xFF);
  }
  
  for(int i=0;i<N;i++){
    uint8_t id = ids[i] % 0x100;
    serialport_writebyte(fd,id);
    uint8_t Lval, Hval;
    break_uint16_to_uint8(val[i],&Lval,&Hval);
    serialport_writebyte(fd,Lval);
    serialport_writebyte(fd,Hval);
  }
  return true;
}

bool getVal(std::vector<int>& ids, std::vector<double>& position, std::vector<double>& velocity, std::vector<double>& torque){
  if( fd == -1 ) error("serial port not opened");
  static int HEADER_FOUND = 0;
  
  if(HEADER_FOUND < HEADER_SIZE){
    print("Searching for header, HEADER_FOUND = ");
    println(HEADER_FOUND);
    
    int MAX_TRY_PER_LOOP = 10;
    static uint8_t b[1];
    int tries_this_time = 0;
    while(serialport_read(fd, b, 1,MAX_TRY_PER_LOOP) != -1 && tries_this_time++<MAX_TRY_PER_LOOP){
      if (b[0] == 0xFF){
        HEADER_FOUND++;
      }
      else {
        HEADER_FOUND = 0;
      }
    }
  }
  
  const int  NUM_DATA  = 7;
  int  NUM_ACTUATORS = ids.size();
  if(HEADER_FOUND == HEADER_SIZE){
    static int recieved_value;

    static int servo_index = 0;
    static int data_index = 0;
    print("Searching for DATA, servo_index = ");
    print(servo_index);
    print(", data_index = ");
    println(data_index);
    
    int MAX_TRY_PER_LOOP = 10;
    static uint8_t vals[NUM_DATA];
    uint8_t b[1];
    while(serialport_read(fd, b, 1,MAX_TRY_PER_LOOP) != -1){
      // increment data pointer
      vals[data_index] = b[0];
      data_index++;
      if(data_index == NUM_DATA){
        ids[servo_index] = vals[0];
        int POSITION = concat_uint8_to_uint16(vals[1],vals[2]);
        int SPEED    = concat_uint8_to_uint16(vals[3],vals[4]);
        int LOAD     = concat_uint8_to_uint16(vals[5],vals[6]);
        position[servo_index] = uint16_to_real(POSITION,MAX_POSITION);
        velocity[servo_index] = uint16_to_real(SPEED,MAX_SPEED);
        torque[servo_index] = uint16_to_real(LOAD,MAX_TORQUE);

#ifdef OUTPUT_ASCII
        print("id[ ");
        print(servo_index);
        print(" ]: ");
        print(ids[servo_index]);
        print(" got data: ");
        println(position[servo_index]);
        println(velocity[servo_index]);
        println(torque[servo_index]);
#endif
        servo_index++;
        data_index = 0;
        if(servo_index == NUM_ACTUATORS){
          servo_index = 0;
          HEADER_FOUND = 0;
          return true;
        }
      }
    }
  }
  return false;
}