#ifndef tldpSerial_HPP
#define tldpSerial_HPP

#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <iomanip>

/* change this definition for the correct port */
#define _POSIX_SOURCE 1 /* POSIX compliant source */

#define FALSE 0
#define TRUE 1

class tldpSerial
{
protected:
  int fd, rdlen;
  struct termios oldtio,newtio;
public:
  tldpSerial(const char* serialdevice);
  int begin(const int BAUDRATE);
  int receive(char *buf, const unsigned int bufsize);
  int close();
};

#endif // tldpSerial_HPP
