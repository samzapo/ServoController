#include "ServoDriver.h"

#include <stdio.h>    // Standard input/output definitions
#include <stdlib.h>
#include <string.h>   // String function definitions
#include <unistd.h>   // for usleep()
#include <assert.h>
#include <sstream>

enum Parameter{
  P_POSITION  = 1,
  P_VELOCITY  = 2,
  P_LOAD      = 3,
  P_SPECIAL   = 4
};

enum Inst{
  INST_PING       = 1   ,
  INST_READ       = 2   ,
  INST_WRITE      = 3   ,
  INST_REG_WRITE  = 4   ,
  INST_ACTION     = 5   ,
  INST_RESET      = 6   ,
  INST_SYNC_WRITE = 0x83
};


const unsigned char C_READ       = INST_READ        % 0x100;
const unsigned char C_WRITE      = INST_WRITE       % 0x100;
const unsigned char C_SYNC_WRITE = INST_SYNC_WRITE  % 0x100;

const unsigned char C_POSITION = P_POSITION % 0x100;
const unsigned char C_VELOCITY = P_VELOCITY % 0x100;
const unsigned char C_LOAD     = P_LOAD     % 0x100;
const unsigned char C_SPECIAL  = P_SPECIAL  % 0x100;

enum Inds{
 TYPE_INDEX      = 0,
 PARAMETER_INDEX = 1,
 N_INDEX         = 2,
 L_INDEX         = 3,
 HEADER_SIZE     = 4
};


void error(char* msg)
{
  fprintf(stderr, "%s\n",msg);
  exit(EXIT_FAILURE);
}

const int buf_max = 256;

int fd = -1;
int baudrate = 1000000;  // default
char quiet=0;
char eolchar = '\n';
int timeout = 10;
char buf[buf_max];
int rc,n;


/////////////////////////////////////////////////////////////////////////
///////////////////////////  Serial Handling ////////////////////////////
#include "arduino-serial-lib.h"


/* HEADER:
 *  REQUEST_TYPE, PARAMETER, N_IDS, L_SIZE
 */

void Send(const char* out_buf){
#ifndef NDEBUG
  int nbytes_header = HEADER_SIZE;
  
  // Check Header
  printf("Message header:\n");
  for (int i=0; i<nbytes_header-1; i++) {
    printf(" %02x",out_buf[i]);
  }
  printf("\n\n");
#endif
  
  int N_read = out_buf[N_INDEX];
  int L_read = out_buf[L_INDEX]+1;
  
#ifndef NDEBUG
  printf("Message body:\n");
  for (int i=0;i<N_read; i++) {
    for (int j=0;j<L_read; j++) {
      printf(" %02x",out_buf[nbytes_header+i*L_read+j]);
    }
    printf("\n");
  }
  printf("END\n");
#endif
  rc = serialport_write(fd, out_buf);
}

void Recieve(char* in_buf){
  int nbytes_header = HEADER_SIZE;
  // Read Header
  rc = serialport_read(fd, buf, nbytes_header, buf_max, timeout);
  
#ifndef NDEBUG
  // Check Header
  printf("Message header:\n");
  for (int i=0; i<nbytes_header-1; i++) {
    printf(" %02x",buf[i]);
  }
  printf("\n\n");
#endif
  
  int N_read = buf[N_INDEX];
  int L_read = buf[L_INDEX]+1;
  
  int nbytes_body = N_read * L_read;
  rc = serialport_read(fd, &buf[nbytes_header], nbytes_body, buf_max-nbytes_header, timeout);
  
  
#ifndef NDEBUG
  printf("Message body:\n");
  for (int i=0;i<N_read; i++) {
    for (int j=0;j<L_read; j++) {
      printf(" %02x",buf[nbytes_header+i*L_read+j]);
    }
    printf("\n");
  }
  printf("END\n");
#endif
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////  Data Parsing ////////////////////////////

template <class T>
void data2vector(const char * in_buf,const std::vector<int> ids,std::vector<T> val){
  T val_array[0xFF];
  
  int N = in_buf[N_INDEX];
  int L = in_buf[L_INDEX]+1;
  int size_of_values = sizeof(T);

  // now we only support single values in this function
  assert(size_of_values == L-1);
  
  for (int i=0;i<N; i++) {
    val_array[in_buf[HEADER_SIZE+i*L]] = *( (T*) &in_buf[HEADER_SIZE+i*L+1]);
  }
  
  for (int i=0;i<ids.size(); i++) {
    val[i] = val_array[ids[i]];
  }
}

template <class T>
void data2vecvec(const char * in_buf,const std::vector<int> ids,std::vector<std::vector<T> > val){
  std::vector<T> val_array[0xFF];
  
  int N = in_buf[N_INDEX];
  int L = in_buf[L_INDEX]+1;
  int size_of_values = sizeof(T);
  
  // For now we only do single values
  int num_vals_per_id = (L-1) / size_of_values ;
  for (int i=0;i<N; i++) {
    for (int j=0,k=0;j<num_vals_per_id; j+=size_of_values,k+=1) {
      val_array[in_buf[HEADER_SIZE+i*L]].push_back(*( (T*) &in_buf[HEADER_SIZE+i*L+1+j]));
    }
  }

  for (int i=0;i<ids.size(); i++) {
    val[i] = val_array[ids[i]];
  }
}

/// Fills 'ids' with ids of servos that are accessible
bool ServoDriver::init(const char* sp,std::vector<int> ids){
  
  if( fd!=-1 ) {
    serialport_close(fd);
    if(!quiet) printf("closed port %s\n");
  }
  fd = serialport_init(sp, baudrate);
  if( fd==-1 ) error("couldn't open port");
  if(!quiet) printf("opened port %s\n",sp);
  serialport_flush(fd);
  
  return true;
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////  Control Handling ////////////////////////////

bool ServoDriver::setPos(const std::vector<int> ids, const std::vector<int> val){
  if( fd == -1 ) error("serial port not opened");
 // TODO: Tell servo to set desired POSITION of each servo in 'ids' to each positon in 'val'
  
  std::stringstream ss;
  
  unsigned char N = ids.size() % 0x100;
  unsigned char L = 0x00;
  
  ss << C_READ << C_POSITION << N << L;
  for(int i=0;i<ids.size();i++){
    unsigned char i1 = ids[i] % 0x100;
    unsigned char v1 = val[i] % 0x100;
    unsigned char v2 = val[i] / 0x100;
    ss << i1 << v1 << v2;
  }
  
  Send(ss.str().c_str());
  
  return true;
}

bool ServoDriver::getPos(const std::vector<int> ids, std::vector<int> val){
  if( fd == -1 ) error("serial port not opened");
 // TODO: Get POSITION of each servo in 'ids' and set 'val'
  std::stringstream ss;
  
  unsigned char N = ids.size() % 0x100;
  unsigned char L = 0x00;
  
  ss << C_READ << C_POSITION << N << L;
  for(int i=0;i<ids.size();i++){
    unsigned char i1 = ids[i] % 0x100;
    ss << i1;
  }
  
  Send(ss.str().c_str());
  // Write Header
  Recieve(buf);
  
  data2vector<int>(buf,ids,val);
  
  return true;
}

bool ServoDriver::getVel(const std::vector<int> ids, std::vector<int> val){
  if( fd == -1 ) error("serial port not opened");
 // TODO: Get VELOCITY of each servo in 'ids' and set 'val'
  std::stringstream ss;
  
  unsigned char N = ids.size() % 0x100;
  unsigned char L = 0x00;
  
  ss << C_READ << C_VELOCITY << N << L;
  for(int i=0;i<ids.size();i++){
    unsigned char i1 = ids[i] % 0x100;
    ss << i1;
  }
  
  Send(ss.str().c_str());
  // Write Header
  Recieve(buf);
  
  data2vector<int>(buf,ids,val);
  
  return true;
}

bool ServoDriver::setVel(const std::vector<int> ids, std::vector<int> val){
  if( fd == -1 ) error("serial port not opened");
 // TODO: Set desired VELOCITY of each servo in 'ids' to each velocity in 'val'
  std::stringstream ss;
  
  unsigned char N = ids.size() % 0x100;
  unsigned char L = 0x00;
  
  ss << C_READ << C_VELOCITY << N << L;
  for(int i=0;i<ids.size();i++){
    unsigned char i1 = ids[i] % 0x100;
    unsigned char v1 = val[i] % 0x100;
    unsigned char v2 = val[i] / 0x100;
    ss << i1 << v1 << v2;
  }
  
  Send(ss.str().c_str());

  return true;
}

bool ServoDriver::getLoad(const std::vector<int> ids, std::vector<int> val){
  if( fd == -1 ) error("serial port not opened");
 // TODO: Get LOAD of each servo in 'ids' and set 'val'
  std::stringstream ss;

  unsigned char N = ids.size() % 0x100;
  unsigned char L = 0x00;

  ss << C_READ << C_LOAD << N << L;
  for(int i=0;i<ids.size();i++){
    unsigned char i1 = ids[i] % 0x100;
    ss << i1;
  }
  
  Send(ss.str().c_str());
  // Write Header
  Recieve(buf);
  
  data2vector<int>(buf,ids,val);
  
  return true;
}

bool ServoDriver::setLoad(const std::vector<int> ids, std::vector<int> val){
  if( fd == -1 ) error("serial port not opened");
 // TODO: Set desired LOAD of each servo in 'ids' to each load in 'val'
  std::stringstream ss;
  
  unsigned char N = ids.size() % 0x100;
  unsigned char L = 0x00;
  
  ss << C_READ << C_LOAD << N << L;
  for(int i=0;i<ids.size();i++){
    unsigned char i1 = ids[i] % 0x100;
    unsigned char v1 = val[i] % 0x100;
    unsigned char v2 = val[i] / 0x100;
    ss << i1 << v1 << v2;
  }
  
  Send(ss.str().c_str());

  return true;
}

#include "arduino-serial-lib.c"