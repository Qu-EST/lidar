#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */
#include <stdlib.h>

int main()
{
  int fd;
  struct termios SerialPortSettings;
  char write_buffer[] = "A"; 
  int  bytes_written  =  0 ;
  
  fd = open("/dev/ttyACM1",O_RDWR | O_NOCTTY);
  if(fd == -1){
     printf("\n  Error! in Opening ttyUSB0\n");
     exit(-1);
  }
  printf("\n  ttyUSB0 Opened Successfully\n");
  //get the port setting
  tcgetattr(fd, &SerialPortSettings);
  //set the i/o speed
  cfsetispeed(&SerialPortSettings,B115200);
  cfsetospeed(&SerialPortSettings,B115200);
  //set no parity
  SerialPortSettings.c_cflag &= ~PARENB;
  //set the stop bit =1
  SerialPortSettings.c_cflag &= ~CSTOPB;
  //set the data bits = 8
  SerialPortSettings.c_cflag &= ~CSIZE; /* Clears the Mask       */
  SerialPortSettings.c_cflag |=  CS8;   /* Set the data bits = 8 */
  //turnoff hardware based flow contrl
  SerialPortSettings.c_cflag &= ~CRTSCTS;
  //turn on the reader
  SerialPortSettings.c_cflag |= CREAD | CLOCAL;
  //turn off the software flow control
  SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
  SerialPortSettings.c_cflag |= CREAD | CLOCAL;//enable receiver
                                                              
  /* Setting Time outs */                                       
  SerialPortSettings.c_cc[VMIN]  = 10; /* Read 10 characters */  
  SerialPortSettings.c_cc[VTIME] = 0;  /* Wait indefinitely   */


  tcsetattr(fd,TCSANOW,&SerialPortSettings);
  bytes_written = write(fd,write_buffer,sizeof(write_buffer));
  unsigned char read_buffer[32];
  size_t bytes_read;
  for(int i=0;i<20;i++){
    bytes_read = read(fd,&read_buffer,1);
    printf("the read stuff is %s\n", read_buffer);
  }
  
  close(fd);
  return 0;
}
