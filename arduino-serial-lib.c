//
// arduino-serial-lib -- simple library for reading/writing serial ports
//
// 2006-2013, Tod E. Kurt, http://todbot.com/blog/
//

#include "arduino-serial-lib.h"

#include <stdio.h>    // Standard input/output definitions 
#include <unistd.h>   // UNIX standard function definitions 
#include <fcntl.h>    // File control definitions 
#include <errno.h>    // Error number definitions 
#include <termios.h>  // POSIX terminal control definitions 
#include <string.h>   // String function definitions 
#include <sys/ioctl.h>

// uncomment this to debug reads
//#define SERIALPORTDEBUG 

// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
int serialport_init(const char* serialport, int baud)
{
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
  
#ifdef ONE_METHOD
  struct termios toptions;
  int fd;
  
  //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
  fd = open(serialport, O_RDWR | O_NONBLOCK );
  
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
  
  return fd;
#else
  int fd;
  
  /* Open the file descriptor in non-blocking mode */
  fd = open(serialport, O_RDWR | O_NOCTTY);
  
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
  
  return fd;
#endif
}

//
int serialport_close( int fd )
{
    return close( fd );
}

//
int serialport_writebyte( int fd, uint8_t b)
{
    int n = write(fd,&b,1);
    if( n!=1)
        return -1;
    return 0;
}

//
int serialport_write(int fd, const uint8_t* str, int len )
{
  if (len < 1) {
    printf("serialport_write: couldn't write buffer of size %d\n",len);
    exit(1);
  }
    int n = write(fd, str, len);
    if( n!=len ) {
        printf("serialport_write: couldn't write string: '%s'\n",str);
        perror("serialport_write: couldn't write whole string\n");
        return -1;
    }
    return 0;
}

//
int serialport_read_until(int fd, uint8_t* buf, uint8_t until, int buf_max, int timeout)
{
#ifdef SERIALPORTDEBUG
 printf("serialport_read_until(fd = %d,uint8_t*,until = %02x, buf_max = %d, timeout = %d)\n",fd,until,buf_max,timeout);
#endif

    uint8_t b[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    do { 
        int n = read(fd, b, 1);  // read a uint8_t at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 1 * 1000 );  // wait 1 msec try again
            timeout--;
            continue;
        }
#ifdef SERIALPORTDEBUG  
        printf("serialport_read_until: i=%d, n=%d b='%c'\n",i,n,b[0]); // debug
#endif
      fprintf(stdout,"%02x ",b[0]);
        buf[i] = b[0];
        i++;
    } while( b[0] != until && i < buf_max && timeout>0 );

    buf[i] = 0;  // null terminate the string
    return 0;
}

int serialport_read(int fd, uint8_t* buf, int nbytes, int buf_max,int timeout){
#ifdef SERIALPORTDEBUG
  printf("serialport_read(fd = %d,uint8_t*, buf_max = %d, timeout = %d)\n",fd,buf_max,timeout);
#endif
  
  int n = read(fd, buf, nbytes);
#ifdef SERIALPORTDEBUG
  printf("%i bytes got read...\n", n);
  // print what's in the buffer 
  printf("Buffer contains...\n%s\n", buf);
#endif
  
  assert(n!=-1);    // couldn't read at all
  assert(n!=nbytes);// couldn't read desired number of bytes
  return n;
}

//
int serialport_flush(int fd)
{
    sleep(2); //required to make flush work, for some reason
    return tcflush(fd, TCIOFLUSH);
}
