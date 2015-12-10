#include <stdio.h>    // Standard input/output definitions 
#include <stdint.h>    // uint*_t defs 

#include <unistd.h>   // UNIX standard function definitions 
#include <fcntl.h>    // File control definitions 
#include <errno.h>    // Error number definitions 
#include <termios.h>  // POSIX terminal control definitions 
#include <string.h>   // String function definitions 
#include <sys/ioctl.h>

int main(int argc, char* argv[])
{
  const char* serialport = "/dev/ttyUSB0";
  int baud = 115200; 
  int fd;
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
  
#ifndef ONE_METHOD
  struct termios toptions;
  
  fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
  //fd = open(serialport, O_RDWR | O_NONBLOCK );
  
  if (fd == -1)  {
    perror("serialport_init: Unable to open port ");
    return -1;
  }
  
  int iflags = TIOCM_DTR;
  ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
  //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR
  
  if (tcgetattr(fd, &toptions) < 0) {
    perror("serialport_init: Couldn't get term attributes");
    return -1;
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
  
  tcsetattr(fd, TCSANOW, &toptions);
  if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
    perror("init_serialport: Couldn't set term attributes");
    return -1;
  }
  
#else
  
  /* Open the file descriptor in non-blocking mode */
  fd = open(serialport, O_RDWR | O_NOCTTY | O_NONBLOCK );
  
  /* Set up the control structure */
  struct termios toptions;
  
  /* Get currently set options for the tty */
  tcgetattr(fd, &toptions);
  
  /* Set custom options */
  
  /* 9600 baud */
  cfsetispeed(&toptions, brate);
  cfsetospeed(&toptions, brate);
  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  /* no hardware flow control */
  toptions.c_cflag &= ~CRTSCTS;
  /* enable receiver, ignore status lines */
  toptions.c_cflag |= CREAD | CLOCAL;
  /* disable input/output flow control, disable restart chars */
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
  /* disable canonical input, disable echo,
   disable visually erase chars,
   disable terminal-generated signals */
  toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /* disable output processing */
  toptions.c_oflag &= ~OPOST;
  
  /* wait for 24 characters to come in before read returns */
  toptions.c_cc[VMIN] = 12;
  /* no minimum time to wait before read returns */
  toptions.c_cc[VTIME] = 0;
  
  /* commit the options */
  tcsetattr(fd, TCSANOW, &toptions);
  
  /* Wait for the Arduino to reset */
  usleep(1000*1000);
  /* Flush anything already in the serial buffer */
  tcflush(fd, TCIFLUSH);
  
#endif

  uint8_t b[4] = {0x06, 0x00, 0x00, 0x00};
  int n1 = write(fd,&b,4);
  

  uint8_t b_in[1] = {0x00};
  int n0 = read(fd,&b_in, 1);  // read a uint8_t at a time

  printf("%02x ",b_in[0]);

  return close( fd );
}
