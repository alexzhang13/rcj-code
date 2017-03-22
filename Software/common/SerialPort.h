#ifndef _SerialPort_h_
#define _SerialPort_h_
#include <stdio.h>
#include <stdint.h>

struct termios;

class SerialPort {
 public:
    SerialPort(const char *devName=0,uint32_t rate=0);
    virtual ~SerialPort();

    bool open(const char *devName,uint32_t rate);
    void close(void);

    // DataPort function
    virtual int printf(const char *fmt,...);
    virtual int vprintf(const char *fmt,va_list ap);

    virtual int scanf(const char *fmt,...);
    virtual int vscanf(const char *fmt,va_list ap);

    virtual char *fgets(char *buf,int size);
    virtual int  fputs(char *buf);

    virtual int read(char *buf,int size);
    virtual int write(char *buf,int size);

 protected:
    int mFdRd,mFdWr;
    FILE *mFileFdRd, *mFileFdWr;
    struct termios *mTioOrg,*mTio;
}; // class SerialPort

#endif // _SerialPort_h_
