#ifndef _Thread_h_
#define _Thread_h_
#include <pthread.h>

class Thread {
public:
    Thread(int priority=50, int stackSize=20000);

    bool m_SuspendFlag;

    virtual bool start(void);
    virtual void run(void)=0;

    virtual void pauseThread();
    virtual void unpauseThread();
    virtual void checkSuspend();
    virtual void DestroyThread();

    inline bool isExit() {
        return mExitFlag;
    }

protected:
    int mPriority, mStackSize;

private:
    pthread_t mThread;
    pthread_mutex_t m_SuspendMutex;
    pthread_cond_t m_ResumeCond;
    bool mExitFlag;

    static void *threadFunc(void * obj);
}; // class Thread
#endif // _Thread_h_
