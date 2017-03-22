#ifndef _UartRx_h_
#define _UartRx_h_

#include "Thread.h"
#include "SerialPort.h"

class UartRx : public Thread {
 public:
    UartRx(SerialPort *port)
        :mPort(port)
    {}

    virtual void run(void);
 protected:
    SerialPort *mPort;
};
#endif // _UartRx_h_
