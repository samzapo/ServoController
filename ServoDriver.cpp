#include "ServoDriver.h"

#include <iostream>
#include <stdio.h>    // Standard input/output definitions
#include <stdlib.h>
#include <string.h>   // String function definitions
#include <unistd.h>   // for usleep()
#include <assert.h>
#include <sstream>

void error(char* msg)
{
  fprintf(stderr, "%s\n",msg);
//  exit(EXIT_FAILURE);
}

const int buf_max = 0xFF;

int fd = -1;
int baudrate = 115200;  // default
uint8_t quiet=1;
int rc,n;


/////////////////////////////////////////////////////////////////////////
///////////////////////////  Serial Handling ////////////////////////////
#include "arduino-serial-lib.h"


/* HEADER:
 *  REQUEST_TYPE, PARAMETER, N_IDS, L_SIZE
 */

void Send(uint8_t * out_buf,int timeout = 10){
  fprintf(stdout, "Send:\n");
#ifndef NDEBUG
  // Check Header
  for (int i=0; i<HEADER_SIZE; i++) {
    fprintf(stdout, "%02x ",out_buf[i]);
  }
  fprintf(stdout, "\n");
#endif
  
  int N = out_buf[N_INDEX];
  int L = out_buf[L_INDEX];
  
#ifndef NDEBUG
  for (int i=0;i<N; i++) {
    for (int j=0;j<L+1; j++) {
      fprintf(stdout, "%02x ",out_buf[HEADER_SIZE+i*(L+1)+j]);
    }
    fprintf(stdout, "   ");
  }
  fprintf(stdout, "\n");
#endif
  int message_size = HEADER_SIZE + N*L;
  rc = serialport_write(fd, out_buf,message_size);
}

void Recieve(uint8_t* in_buf,int timeout = 10){
  fprintf(stdout, "Recieve:\n");

  // Read Header
  rc = serialport_read(fd, in_buf, HEADER_SIZE, buf_max, timeout);
  
#ifndef NDEBUG
  for (int i=0; i<HEADER_SIZE; i++) {
    fprintf(stdout, "%02x ",in_buf[i]);
  }
  fprintf(stdout, "\n");
#endif
  
  int N = in_buf[N_INDEX];
  int L = in_buf[L_INDEX];
  
  int nbytes_body = N * (L+1);
  rc = serialport_read(fd, &in_buf[HEADER_SIZE], nbytes_body, buf_max-HEADER_SIZE, timeout);
  
  
#ifndef NDEBUG
  for (int i=0;i<N; i++) {
    for (int j=0;j<(L+1); j++) {
      fprintf(stdout, "%02x ",in_buf[HEADER_SIZE+i*(L+1)+j]);
    }
    fprintf(stdout, "   ");
  }
  fprintf(stdout, "\n");
#endif
}
bool ping(){
  if( fd == -1 ) error("serial port not opened");
  // TODO: Get POSITION of each servo in 'ids' and set 'val'
  
  uint8_t N = 0;
  const uint8_t L = 0x00;
  
  uint8_t buf[buf_max*buf_max];
  buf[0] = INST_PING ;
  buf[1] = P_EMPTY;
  buf[2] = N;
  buf[3] = L;
  
  Send(buf);
  
  uint8_t in_buf[buf_max*buf_max];
  Recieve(in_buf);
  
  assert(in_buf[4] == 1);
  
  return true;
}

/// Fills 'ids' with ids of servos that are accessible
bool init(const char* sp,std::vector<int> ids){
  
  if( fd!=-1 ) {
    serialport_close(fd);
    if(!quiet) fprintf(stdout, "closed port %s\n");
  }
  fd = serialport_init(sp, baudrate);
  if( fd==-1 ) error("couldn't open port");
  if(!quiet) fprintf(stdout, "opened port %s\n",sp);
  serialport_flush(fd);
  
  return true;
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////  Data Parsing ////////////////////////////

template <class T>
void data2vector(const uint8_t * in_buf,const std::vector<int>& ids,std::vector<T>& val){
  T val_array[0xFF];
  
  int N = in_buf[N_INDEX];
  int size_of_values = sizeof(T);
  int L = size_of_values;//in_buf[L_INDEX];
  fprintf(stdout, "Parsing Message:\n");
  fprintf(stdout, "N: %02x\n",N);
  fprintf(stdout, "L: %02x\n",L);
  fprintf(stdout, "size_of_values: %02x\n",size_of_values);

  // now we only support single values in this function
  //if(size_of_values != L-1)
  //  error("Data being imported is the wrong size for this type!");
  
  for (int i=0;i<N; i++) {
    uint8_t ind = in_buf[HEADER_SIZE+i*(L+1)];
    memcpy(&val_array[ind],&in_buf[HEADER_SIZE+i*(L+1)+1],size_of_values);
    fprintf(stdout, "%02x : ",ind);
    fprintf(stdout, "%d ",val_array[ind]);
    fprintf(stdout, "\n");
  }
  fprintf(stdout, "\n");
  
  for (int i=0;i<ids.size(); i++) {
    val[i] = val_array[ids[i]];
  }

}

template <class T>
void data2vecvec(const uint8_t * in_buf,const std::vector<int>& ids,std::vector<std::vector<T> >& val){
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

/////////////////////////////////////////////////////////////////////////
///////////////////////////  Control Handling ////////////////////////////

namespace LocalTemplatedFcn {
template<class T>
bool setVal(const std::vector<int>& ids, const Parameter type, const std::vector<T>& val){
  if( fd == -1 ) error("serial port not opened");
 // TODO: Tell servo to set desired POSITION of each servo in 'ids' to each positon in 'val'
  
  uint8_t N = ids.size() % 0x100;
  const uint8_t L = sizeof(T);
  
  uint8_t buf[buf_max*buf_max];
  buf[0] = INST_WRITE % 0x100;
  buf[1] = (int)type % 0x100;
  buf[2] = N;
  buf[3] = L;

  for(int i=0;i<N;i++){
    uint8_t id = ids[i] % 0x100;
    buf[HEADER_SIZE + i*(L+1)] = id;
    memcpy( &buf[HEADER_SIZE + i*(L+1) + 1] , &val[i], L);
  }
  
  Send(buf);
  
  std::vector<T> val2(ids.size());
  data2vector<T>(buf,ids,val2);

  return true;
}

template<class T>
bool getVal(const std::vector<int>& ids, const Parameter type,  std::vector<T>& val){
  if( fd == -1 ) error("serial port not opened");
 // TODO: Get POSITION of each servo in 'ids' and set 'val'
  
  uint8_t N = ids.size() % 0x100;
  const uint8_t L = 0x00;
  
  uint8_t buf[buf_max*buf_max];
  buf[0] = INST_READ % 0x100;
  buf[1] = (int)type % 0x100;
  buf[2] = N;
  buf[3] = L;
  
  for(int i=0;i<N;i++){
    uint8_t id = ids[i] % 0x100;
    buf[HEADER_SIZE + i*(L+1)] = id;
  }
  
  Send(buf);
  
  uint8_t in_buf[buf_max*buf_max];
  Recieve(in_buf);
  
  data2vector<T>(in_buf,ids,val);
  
  return true;
}
}

  template<>
  bool setVal<uint16_t>(const std::vector<int>& ids, const Parameter type, const std::vector<uint16_t>& val){
    return LocalTemplatedFcn::setVal<uint16_t>(ids,type,val);
  }
  
  template<>
  bool getVal<uint16_t>(const std::vector<int>& ids, const Parameter type, std::vector<uint16_t>& val){
    return LocalTemplatedFcn::getVal<uint16_t>(ids,type,val);
  }
  
  // CHAR
  template<>
  bool setVal<uint8_t>(const std::vector<int>& ids, const Parameter type, const std::vector<uint8_t>& val){
    return LocalTemplatedFcn::setVal<uint8_t>(ids,type,val);
  }
  
  template<>
  bool getVal<uint8_t>(const std::vector<int>& ids, const Parameter type, std::vector<uint8_t>& val){
    return LocalTemplatedFcn::getVal<uint8_t>(ids,type,val);
  }
  
  
  // FLOAT (32 bit)
  template<>
  bool setVal<float>(const std::vector<int>& ids, const Parameter type, const std::vector<float>& val){
    return LocalTemplatedFcn::setVal<float>(ids,type,val);
  }
  
  template<>
  bool getVal<float>(const std::vector<int>& ids, const Parameter type, std::vector<float>& val){
    return LocalTemplatedFcn::getVal<float>(ids,type,val);
  }
  
  // DOUBLE (64 bit)
  template<>
  bool setVal<double>(const std::vector<int>& ids, const Parameter type, const std::vector<double>& val){
    return LocalTemplatedFcn::setVal<double>(ids,type,val);
  }
  
  template<>
  bool getVal<double>(const std::vector<int>& ids, const Parameter type, std::vector<double>& val){
    return LocalTemplatedFcn::getVal<double>(ids,type,val);
  }
  
  // INT (32 bit)
  template<>
  bool setVal<int>(const std::vector<int>& ids, const Parameter type, const std::vector<int>& val){
    return LocalTemplatedFcn::setVal<int>(ids,type,val);
  }
  
  template<>
  bool getVal<int>(const std::vector<int>& ids, const Parameter type, std::vector<int>& val){
    return LocalTemplatedFcn::getVal<int>(ids,type,val);
  }


#include "arduino-serial-lib.c"
