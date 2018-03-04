#include "../_headers/Thread.h"

#include <string.h>
#include <stdio.h>
#include <errno.h>

Thread::Thread(int priority, int stackSize)
    :mPriority(priority), mStackSize(stackSize)
{
    start();
}

bool Thread::start(void){
    int status;

    pthread_attr_t attr;
    status = pthread_attr_init(&attr);
    if(status != 0)
        return false;

    // set stack size
    status = pthread_attr_setstacksize(&attr,mStackSize);
    if(status!=0)
        printf("Thread::error: set stacksize fails(%s)\n",strerror(status));

    // set policy
    status = pthread_attr_setschedpolicy(&attr,SCHED_FIFO);
    if(status!=0)
        printf("Thread::error: set policy fails(%d)\n",status);

    // set priority
    struct sched_param param;
    memset(&param,0,sizeof(param));
    param.sched_priority = mPriority;
    status = pthread_attr_setschedparam(&attr,&param);
    if(status!=0)
        printf("Thread::error: set priority fails(%s)\n",strerror(status));

    // create thread
    status = pthread_create(&mThread,&attr,threadFunc,this);
    if(status != 0){
        printf("Thread::error: create thread fails(%s)\n",strerror(status));
        return false;
    }

    pthread_attr_destroy(&attr);
    return true;
}


void *Thread::threadFunc(void * obj){
    ((Thread*)obj)->run();
    return NULL;
}

void Thread::pauseThread()
{ // tell the thread to suspend
    pthread_mutex_lock(&m_SuspendMutex);
    m_SuspendFlag = 1;
    pthread_mutex_unlock(&m_SuspendMutex);
}
void Thread::unpauseThread()
{ // tell the thread to resume
    pthread_mutex_lock(&m_SuspendMutex);
    m_SuspendFlag = 0;
    //phtread_cond_broadcast(&m_ResumeCond);
    pthread_mutex_unlock(&m_SuspendMutex);
}
void Thread::checkSuspend()
{ // if suspended, suspend until resumed
    pthread_mutex_lock(&m_SuspendMutex);
    while (m_SuspendFlag != 0) pthread_cond_wait(&m_ResumeCond, &m_SuspendMutex);
    pthread_mutex_unlock(&m_SuspendMutex);
}
