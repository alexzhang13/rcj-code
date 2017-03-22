#ifndef _Thread_h_
#define _Thread_h_
#include <pthread.h>

class Thread {
 public:
    Thread(int priority=50, int stackSize=20000);

    virtual bool start(void);
    virtual void run(void)=0;

 protected:
    int mPriority, mStackSize;

 private:
    pthread_t mThread;

    static void *threadFunc(void * obj);
}; // class Thread
#endif // _Thread_h_
