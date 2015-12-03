#include "ServoDriver.h"

#include <stdio.h>    // Standard input/output definitions
#include <stdlib.h>
#include <string.h>   // String function definitions
#include <unistd.h>   // for usleep()
#include <assert.h>
#include <sstream>
#include <iostream>

enum Inst{
  INST_PING       = 1   ,
  INST_READ       = 2   ,
  INST_WRITE      = 3   ,
  INST_REG_WRITE  = 4   ,
  INST_ACTION     = 5   ,
  INST_RESET      = 6   ,
  INST_SYNC_WRITE = 0x83
};

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
//  exit(EXIT_FAILURE);
}

const int buf_max = 12*12;

uint8_t quiet=1;
uint8_t eolchar = '\n';
int timeout = 10;
uint8_t buf[buf_max];

/////////////////////////////////////////////////////////////////////////
///////////////////////////  Serial Handling ////////////////////////////
#include <SerialStream.h>
#include <iostream>

using namespace LibSerial ;

SerialStream ardu ;

void serialport_open(const char* sp){
  /*The arduino must be setup to use the same baud rate*/
  fprintf(stdout, "Setting Params:\n");
  ardu.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
  ardu.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
  ardu.SetNumOfStopBits(1) ;
  ardu.SetParity( SerialStreamBuf::PARITY_ODD ) ;
  ardu.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_HARD ) ;
  ardu.SetVTime(1);
  fprintf(stdout, "Opening:\n");
  ardu.Open( sp ) ;
  fprintf(stdout, "Opened:\n");

}

void serialport_close(const char* sp){
  /*The arduino must be setup to use the same baud rate*/
  ardu.Close() ;
}


/* HEADER:
 *  REQUEST_TYPE, PARAMETER, N_IDS, L_SIZE
 */
const int BUFFER_SIZE = 256 ;
char input_buffer[BUFFER_SIZE] ;
char output_buffer[BUFFER_SIZE] ;

void Send(uint8_t * out_buf){
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
  memcpy(output_buffer,out_buf,sizeof(uint8_t)*message_size);
  ardu.write( output_buffer, message_size ) ;

}

void Recieve(uint8_t* in_buf){
  // Read Header
  ardu.read( input_buffer, HEADER_SIZE ) ;
  
  for (int i=0; i<HEADER_SIZE; i++)
    memcpy(&in_buf[i],&input_buffer[i],sizeof(uint8_t));
//    in_buf[i] = input_buffer[i];
#ifndef NDEBUG
  // Check Header
  fprintf(stdout, "Message header:\n");
  for (int i=0; i<HEADER_SIZE; i++) {
    fprintf(stdout, " %02x",in_buf[i]);
  }
  fprintf(stdout, "\n\n");
#endif

  int N = buf[N_INDEX];
  int L = buf[L_INDEX]+1;
  
  int nbytes_body = N * L;

  ardu.read( input_buffer, nbytes_body ) ;
  for (int i=0; i<nbytes_body; i++)
    memcpy(&in_buf[i+HEADER_SIZE],&input_buffer[i],sizeof(uint8_t));
//    in_buf[i+HEADER_SIZE] = input_buffer[i];

#ifndef NDEBUG
  fprintf(stdout, "Message body:\n");
  for (int i=0;i<N; i++) {
    for (int j=0;j<L; j++) {
      fprintf(stdout, " %02x",in_buf[HEADER_SIZE+i*L+j]);
    }
    fprintf(stdout, "\n");
  }
  fprintf(stdout, "END\n");
#endif
}

/// Fills 'ids' with ids of servos that are accessible
bool ServoDriver::init(const char* sp,std::vector<int> ids){
#ifndef NDEBUG
  fprintf(stdout, "Opening: %s",sp);
#endif
  serialport_open(sp);
  
  return true;
}
/*
int serialport_init(const char* serialport, int baud)
{
  struct termios toptions;
  int fd;
  
  fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
  //fd = open(serialport, O_RDWR | O_NONBLOCK );
  
  if (fd == -1)  {
    perror("serialport_init: Unable to open port ");
    fprintf(stdout, "serialport_init: Unable to open port %s",serialport);
    return -1;
  }
  
  //int iflags = TIOCM_DTR;
  //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
  //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR
  
  fcntl(fd, F_SETFL, 0); // blocking
  if (tcgetattr(fd, &toptions) < 0) {
    perror("serialport_init: Couldn't get term attributes");
    return -1;
  }
  speed_t brate = baud; // let you override switch below if needed
  switch(baud) {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
#ifdef B14400
    case 14400:  brate=B14400;  break;
#endif
    case 19200:  brate=B19200;  break;
#ifdef B28800
    case 28800:  brate=B28800;  break;
#endif
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
  }
  cfsetispeed(&toptions, brate);
  cfsetospeed(&toptions, brate);
  
  // 8N1
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  // no flow control
  toptions.c_cflag &= ~CRTSCTS;
  
  //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset
  
  toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
  
  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  toptions.c_oflag &= ~OPOST; // make raw
  
  // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
  toptions.c_cc[VMIN]  = 0;
  toptions.c_cc[VTIME] = 0;
  //toptions.c_cc[VTIME] = 20;
  toptions.c_cflag=B57600;
  
  tcsetattr(fd, TCSANOW, &toptions);
  if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
    perror("init_serialport: Couldn't set term attributes");
    return -1;
  }
  
  return fd;
}

*/

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
