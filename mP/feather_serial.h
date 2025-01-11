#include <stdio.h>    // Standard input/output definitions 
#include <stdlib.h>
#include <unistd.h>   // UNIX standard function definitions 
#include <math.h>
// #include "iostream"
#include <sys/time.h>
#include <fcntl.h>    // File control definitions 
#include <errno.h>    // Error number definitions 
#include <termios.h>  // POSIX terminal control definitions 
#include <string.h>   // String function definitions 

/*
Function serialport_init initializes a serial port for communication.
Inputs:
const char* serialport - path to input port e.g. "/dev/ttymxc3"
int baud - baud rate, speed of communication
*/
int serialport_init(const char* serialport, int baud);

/*
Function serialport_close closes serial port communication.
Inputs:
int fd - file descriptor
*/
int serialport_close(int fd);

/*
Function serialport_writebyte writes a single byte 
to a serial port.
Inputs:
int fd - file descriptor
const char* str - command to be written to port
*/
int serialport_writebyte(int fd, const char* str);

/*
Function serialport_write writes to a serial port.
Inputs:
int fd - file descriptor
const char* str - command to be written to port
*/
int serialport_write(int fd, const char* str);

/*
Function serialport_read reads a serial port.
Inputs:
int fd - file descriptor
char* buf - buffer that we read into
int buf_max - maximum buffer length
int timeout - timeout in seconds
*/
int serialport_read(int fd, char* buf, int buf_max, int timeout);

/*
Function serialport_read_until reads a serial port until an end-of
		 line character is reached.
Inputs:
int fd - file descriptor
char* buf - buffer that we read into
int buf_max - maximum buffer length
int timeout - timeout in seconds
*/
int serialport_read_until(int fd, char* buf, int buf_max, char eol, int timeout);
