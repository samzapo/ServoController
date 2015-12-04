#include "ServoDriver.h"

#include <stdio.h>    // Standard input/output definitions
#include <stdlib.h>
#include <string.h>   // String function definitions
#include <unistd.h>   // for usleep()
#include <assert.h>
#include <sstream>

using namespace ServoDriver;
void error(char* msg)
{
  fprintf(stderr, "%s\n",msg);
//  exit(EXIT_FAILURE);
}

const int buf_max = 12*12;

int fd = -1;
int baudrate = 115200;  // default
uint8_t quiet=1;
uint8_t eolchar = '\n';
uint8_t buf[buf_max];
int rc,n;


/////////////////////////////////////////////////////////////////////////
///////////////////////////  Serial Handling ////////////////////////////
#include "arduino-serial-lib.h"


/* HEADER:
 *  REQUEST_TYPE, PARAMETER, N_IDS, L_SIZE
 */

void Send(uint8_t * out_buf,int timeout = 10){
#ifndef NDEBUG
  // Check Header
  fprintf(stdout, "Message header:\n");
  for (int i=0; i<HEADER_SIZE; i++) {
    fprintf(stdout, " %02x",out_buf[i]);
  }
  fprintf(stdout, "\n\n");
#endif
  
  int N = out_buf[N_INDEX];
  int L = out_buf[L_INDEX]+1;
  
#ifndef NDEBUG
  fprintf(stdout, "Message body:\n");
  for (int i=0;i<N; i++) {
    for (int j=0;j<L; j++) {
      fprintf(stdout, " %02x",out_buf[HEADER_SIZE+i*L+j]);
    }
    fprintf(stdout, "\n");
  }
  fprintf(stdout, "END\n");
#endif
  int message_size = HEADER_SIZE + N*L;
  rc = serialport_write(fd, out_buf,message_size);
}

void Recieve(uint8_t* in_buf,int timeout = 10){
  // Read Header
  rc = serialport_read(fd, buf, HEADER_SIZE, buf_max, timeout);
  
#ifndef NDEBUG
  // Check Header
  fprintf(stdout, "Message header:\n");
  for (int i=0; i<HEADER_SIZE; i++) {
    fprintf(stdout, " %02x",buf[i]);
  }
  fprintf(stdout, "\n\n");
#endif
  
  int N = buf[N_INDEX];
  int L = buf[L_INDEX]+1;
  
  int nbytes_body = N * L;
  rc = serialport_read(fd, &buf[HEADER_SIZE], nbytes_body, buf_max-HEADER_SIZE, timeout);
  
  
#ifndef NDEBUG
  fprintf(stdout, "Message body:\n");
  for (int i=0;i<N; i++) {
    for (int j=0;j<L; j++) {
      fprintf(stdout, " %02x",buf[HEADER_SIZE+i*L+j]);
    }
    fprintf(stdout, "\n");
  }
  fprintf(stdout, "END\n");
#endif
}

/// Fills 'ids' with ids of servos that are accessible
bool ServoDriver::init(const char* sp,std::vector<int> ids){
  
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
  int L = in_buf[L_INDEX]+1;
  int size_of_values = sizeof(T);

  // now we only support single values in this function
  if(size_of_values != L-1)
    error("Data being imported is the wrong size for this type!");
  
  for (int i=0;i<N; i++) {
    uint8_t ind = in_buf[HEADER_SIZE+i*L];
    memcpy(&val_array[ind],&in_buf[HEADER_SIZE+i*L+1],size_of_values);
  }
  
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

template<class T>
bool setVal(const std::vector<int>& ids, const ServoDriver::Parameter type, const std::vector<T>& val){
  if( fd == -1 ) error("serial port not opened");
 // TODO: Tell servo to set desired POSITION of each servo in 'ids' to each positon in 'val'
  
  uint8_t N = ids.size() % 0x100;
  const uint8_t L = sizeof(T);
  
  buf[0] = INST_WRITE % 0x100;
  buf[1] = (int)type % 0x100;
  buf[2] = N;
  buf[3] = L;
  
#ifndef NDEBUG
  fprintf(stdout, "Position:\n");
#endif
  for(int i=0;i<N;i++){
    uint8_t v[L];
    memcpy(v, &val[i], L);

    uint8_t i1 = ids[i] % 0x100;
    buf[HEADER_SIZE + i*(L+1)]     = i1;
    for(int j=0;j<L;j++){
      buf[HEADER_SIZE + i*(L+1) + j + 1] = v[j];
    }
    
#ifndef NDEBUG
//    fprintf(stdout, "%d:  (%s) floating point:%f OR integer:%d --> %02x: ",ids[i],typeid(T).name(),val[i],val[i],i1);
    for(int j=0;j<L;j++)
      fprintf(stdout, " %02x",buf[HEADER_SIZE + i*(L+1) + j + 1]);
    fprintf(stdout, "\n");
    
#endif
  }
#ifndef NDEBUG
  fprintf(stdout, "END\n");
#endif
  
  Send(buf);
  
#ifndef NDEBUG
  fprintf(stdout, "Position (re-parsed):\n");
  std::vector<T> val2(ids.size());
  data2vector<T>(buf,ids,val2);
  for(int i=0;i<ids.size();i++){
//    fprintf(stdout, "%d: (%s) floating point:%f OR integer:%d \n",ids[i],typeid(T).name(),val2[i],val2[i]);
  }
  fprintf(stdout, "END\n");
#endif


  return true;
}

template<class T>
bool getVal(const std::vector<int>& ids, const ServoDriver::Parameter type,  std::vector<T>& val){
  if( fd == -1 ) error("serial port not opened");
 // TODO: Get POSITION of each servo in 'ids' and set 'val'
  
  uint8_t N = ids.size() % 0x100;
  const uint8_t L = 0x00;
  
  buf[0] = INST_READ % 0x100;
  buf[1] = (int)type % 0x100;
  buf[2] = N;
  buf[3] = L;
  
#ifndef NDEBUG
  fprintf(stdout, "Load:\n");
#endif
  for(int i=0;i<N;i++){
    uint8_t i1 = ids[i] % 0x100;
    buf[HEADER_SIZE + i*(L+1)]     = i1;
    
#ifndef NDEBUG
    fprintf(stdout, "%d --> %02x\n",ids[i],i1);
#endif
  }
#ifndef NDEBUG
  fprintf(stdout, "END\n");
#endif
  
  Send(buf);
  
  // Write Header
  Recieve(buf);
  
  data2vector<T>(buf,ids,val);
  
  return true;
}

bool ServoDriver::ping(){
  if( fd == -1 ) error("serial port not opened");
  // TODO: Get POSITION of each servo in 'ids' and set 'val'
  
  uint8_t N = 0;
  const uint8_t L = 0x00;
  
  buf[0] = INST_PING ;
  buf[1] = P_EMPTY;
  buf[2] = N;
  buf[3] = L;
  
  Send(buf);
  
  // Write Header
  rc = serialport_read(fd, buf, 1, 1, 1000);
  
  fprintf(stdout, "Response: %d\n",buf[0]);
  assert(buf[0] == 1);
  
  return true;
}


//////////////////////////////////////////////////////////////////////////////
// Template specializations so I can put the big templated function in here //

// SHORT
template<>
bool ServoDriver::setVal<uint16_t>(const std::vector<int>& ids, const Parameter type, const std::vector<uint16_t>& val){
  return ::setVal<uint16_t>(ids,type,val);
}

template<>
bool ServoDriver::getVal<uint16_t>(const std::vector<int>& ids, const Parameter type, std::vector<uint16_t>& val){
  return ::getVal<uint16_t>(ids,type,val);
}

// CHAR
template<>
bool ServoDriver::setVal<uint8_t>(const std::vector<int>& ids, const Parameter type, const std::vector<uint8_t>& val){
  return ::setVal<uint8_t>(ids,type,val);
}

template<>
bool ServoDriver::getVal<uint8_t>(const std::vector<int>& ids, const Parameter type, std::vector<uint8_t>& val){
  return ::getVal<uint8_t>(ids,type,val);
}


// FLOAT (32 bit)
template<>
bool ServoDriver::setVal<float>(const std::vector<int>& ids, const Parameter type, const std::vector<float>& val){
  return ::setVal<float>(ids,type,val);
}

template<>
bool ServoDriver::getVal<float>(const std::vector<int>& ids, const Parameter type, std::vector<float>& val){
  return ::getVal<float>(ids,type,val);
}

// DOUBLE (64 bit)
template<>
bool ServoDriver::setVal<double>(const std::vector<int>& ids, const Parameter type, const std::vector<double>& val){
  return ::setVal<double>(ids,type,val);
}

template<>
bool ServoDriver::getVal<double>(const std::vector<int>& ids, const Parameter type, std::vector<double>& val){
  return ::getVal<double>(ids,type,val);
}

// INT (32 bit)
template<>
bool ServoDriver::setVal<int>(const std::vector<int>& ids, const Parameter type, const std::vector<int>& val){
  return ::setVal<int>(ids,type,val);
}

template<>
bool ServoDriver::getVal<int>(const std::vector<int>& ids, const Parameter type, std::vector<int>& val){
  return ::getVal<int>(ids,type,val);
}
#include "arduino-serial-lib.c"
