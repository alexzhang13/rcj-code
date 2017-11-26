#include "../_headers/SerialPort.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>


SerialPort::SerialPort(const char *devName,uint32_t rate)
    :mTioOrg(0),mTio(0),mFdRd(-1),mFdWr(-1),mFileFdRd(0),mFileFdWr(0)
{
    if(!open(devName,rate))
        ::printf("SerialPort::open %s fails\n",devName);
}

SerialPort::~SerialPort(){
    close();
    if(mTioOrg)
        delete mTioOrg;
    if(mTio)
        delete mTio;
}
    
bool SerialPort::open(const char *devName,uint32_t rate){
    if(devName==0)
        return false;

    int bRate;
    switch(rate){
    case 9600:   bRate = B9600;   break;
    case 115200: bRate = B115200; break;
    default:
        printf("SerialPort unsupported rate %d\n",rate);
        return false;
    }

    int fd;
    fd = ::open(devName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd <0) {
        printf("SerialPort::Open (%s) failed\n",devName);
        return false;
    }
    fcntl(fd,F_SETFL,0);

    if(mTioOrg == 0)
        mTioOrg = new struct termios;
    assert(mTioOrg);
    if(mTio == 0)
        mTio = new struct termios;
    assert(mTio);

    /* save current port settings to both */
    tcgetattr(fd,mTioOrg);
    tcgetattr(fd,mTio);

    /* set baudrate */
    cfsetispeed(mTio,bRate);
    cfsetospeed(mTio,bRate);

    mTio->c_cflag |= CS8 | CLOCAL | CREAD;
    mTio->c_cflag &= ~(PARENB|CSTOPB); //|CSIZE);

    mTio->c_iflag = IGNPAR;
    mTio->c_oflag = 0;
    mTio->c_lflag = 0;
    mTio->c_cc[VMIN]=1;
    mTio->c_cc[VTIME]=0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,mTio);

    mFdRd = mFdWr = fd;
    mFileFdRd = mFileFdWr = ::fdopen(fd,"r+"); // for read and write
    return true;
}

void SerialPort::close(void){
    if(mFdRd>=0)
        tcsetattr(mFdRd,TCSANOW,mTioOrg);

    if(mFileFdRd)
        ::fclose(mFileFdRd);
    if(mFileFdWr && (mFileFdWr != mFileFdRd))
        ::fclose(mFileFdWr);

    if(mFdRd>=0)
        ::close(mFdRd);
    if((mFdWr>=0) && (mFdRd != mFdWr))
        ::close(mFdWr);

    mFileFdWr = mFileFdRd = 0;
    mFdWr = mFdRd = -1;
}

// DataPort function
int SerialPort::printf(const char *fmt,...) {
    if(mFileFdWr==0)
        return 0;
    va_list args;
    int retVal;
    va_start(args,fmt);
    retVal = ::vfprintf(mFileFdWr,fmt,args);
    va_end(args);
    fflush(mFileFdWr);
    return retVal;
}

int SerialPort::vprintf(const char *fmt,va_list ap) {
    if(mFileFdWr==0)
        return 0;
    int retVal = ::vfprintf(mFileFdWr,fmt,ap);
    fflush(mFileFdWr);
    return retVal;
}

int SerialPort::scanf(const char *fmt,...) {
    if(mFileFdRd==0)
        return 0;
    int retVal;
    va_list args;
    va_start(args,fmt);
    retVal = ::vfscanf(mFileFdRd,fmt,args);
    va_end(args);
    return retVal;
}

int SerialPort::vscanf(const char *fmt,va_list ap) {
    if(mFileFdRd==0)
        return 0;
    return ::vfscanf(mFileFdRd,fmt,ap);
}
    
char *SerialPort::fgets(char *buf,int size) {
    if(mFileFdRd<0)
        return 0;
    char *retVal = ::fgets(buf,size,mFileFdRd);
    for(int i = strlen(buf)-1; isspace(buf[i]) && i>=0; --i)
        buf[i] = 0;
    return retVal;
}

int SerialPort::fputs(char *buf) {
    if(mFileFdWr==0)
        return 0;
    return ::fputs(buf,mFileFdWr);
}

int SerialPort::read(char *buf,int size){
    if(mFdRd<0)
        return 0;
    return ::read(mFdRd,buf,size);
}

int SerialPort::write(char *buf,int size){
    if(mFdWr<0)
        return 0;
    return ::write(mFdWr,buf,size);
}
    
