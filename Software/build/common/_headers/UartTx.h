#ifndef _UartTx_h_
#define _UartTx_h_

#include "../_headers/SerialPort.h"
#include "../_headers/Thread.h"

class UartTx : public Thread {
 public:
    UartTx(SerialPort *port)
        :mPort(port)
    {}

    virtual void run(void);
 protected:
    SerialPort *mPort;
};
#endif // _UartTx_h_
