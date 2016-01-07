#include <ServoController/ServoDriver.h>

#include <iostream>
#include <stdio.h>    // Standard input/output definitions
#include <stdlib.h>
#include <string.h>   // String function definitions
#include <unistd.h>   // for usleep()
#include <assert.h>
#include <sstream>
#include <math.h>

//#define OUTPUT_ASCII
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

#ifndef NDEBUG
#define println(str) std::cerr << str<< std::endl
#define print(str)   std::cerr << str
#else
#define println(str) //usleep(1000)
#define print(str) //usleep(1000)
#endif


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

#include "arduino-serial-lib.h"

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

double uint16_to_real(unsigned int uint16_value, double max_value){
  return (( (long int) uint16_value - VAL_FFFF_2) / VAL_FFFF_2) * max_value;
}

unsigned int real_to_uint16(double real_value, double max_value){
  int return_val = ((real_value/max_value) * VAL_FFFF_2) + VAL_FFFF_2;
  return return_val - (return_val % 2);
}

unsigned int concat_uint8_to_uint16(uint8_t L_BYTE, uint8_t H_BYTE){
  return (L_BYTE % 0x100) + (((H_BYTE % 0x100 ) << 8) & 0xFF00);
}

void break_uint16_to_uint8(unsigned int uint16_value,uint8_t * L_BYTE, uint8_t * H_BYTE){
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
    unsigned int ival = real_to_uint16(val[i],MAX_POSITION);
    break_uint16_to_uint8(ival,&Lval,&Hval);
    serialport_writebyte(fd,Lval);
    serialport_writebyte(fd,Hval);
  }
  return true;
}

bool getVal(std::vector<int>& ids, std::vector<double>& position, std::vector<double>& velocity, std::vector<double>& torque){
  if( fd == -1 ) error("serial port not opened");
  static int HEADER_FOUND = 0;
  static int servo_index = 0;
  static int data_index = 0;
  
  const int MAX_TRY_PER_LOOP = 10;
  int tries_header = 0;
  while(HEADER_FOUND < HEADER_SIZE && tries_header++<MAX_TRY_PER_LOOP){
    print("Searching for header, HEADER_FOUND = ");
    println(HEADER_FOUND);
    servo_index = 0;
    data_index = 0; 
    
    static uint8_t b[1];
    if(serialport_read(fd, b, 1,MAX_TRY_PER_LOOP) != -1 ){
      if (b[0] == 0xFF){
        HEADER_FOUND++;
      } else {
        HEADER_FOUND = 0;
      }
    }
  }
    
  print("HEADER_FOUND = ");
  println(HEADER_FOUND);
  
  const int  NUM_DATA  = 3;
  int  NUM_ACTUATORS = ids.size();
  int tries_body = 0; 
  while(HEADER_FOUND == HEADER_SIZE && tries_body++<MAX_TRY_PER_LOOP){
#ifdef OUTPUT_ASCII
    print("Searching for DATA, servo_index = ");
    print(servo_index);
    print(", data_index = ");
    println(data_index);
#endif  
    static uint8_t vals[NUM_DATA];
    uint8_t b[1];
    if(serialport_read(fd, b, 1,MAX_TRY_PER_LOOP) != -1){
      print("servo_index = ");
      print(servo_index);
      print(", data_index = ");
      println(data_index);
      // increment data pointer
      vals[data_index] = b[0];
      data_index++;
      if(data_index == NUM_DATA){
        ids[servo_index] = vals[0];
        unsigned int POSITION = concat_uint8_to_uint16(vals[1],vals[2]);
        //unsigned int SPEED    = concat_uint8_to_uint16(vals[3],vals[4]);
        //unsigned int LOAD     = concat_uint8_to_uint16(vals[5],vals[6]);
        position[servo_index] = uint16_to_real(POSITION,MAX_POSITION);
        //velocity[servo_index] = uint16_to_real(SPEED,MAX_SPEED);
        //torque[servo_index] = uint16_to_real(LOAD,MAX_TORQUE);

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
          HEADER_FOUND = 0;
          return true;
        }
      }
    }
  }
  return false;
}
