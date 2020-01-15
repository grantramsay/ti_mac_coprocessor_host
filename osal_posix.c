#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/time.h>

#include "osal_port.h"

void osal_thread_create(osal_thread_t *thread, osal_thread_func_t func) {
    pthread_create(thread, NULL, func, NULL);
}

void osal_thread_join(osal_thread_t *thread) {
    pthread_join(*thread, NULL);
}

void osal_mutex_init(osal_mutex_t *mutex) {
    pthread_mutex_init(mutex, NULL);
}

void osal_mutex_lock(osal_mutex_t *mutex) {
    pthread_mutex_lock(mutex);
}

void osal_mutex_unlock(osal_mutex_t *mutex) {
    pthread_mutex_unlock(mutex);
}

void osal_semaphore_init(osal_semaphore_t *semaphore) {
#if defined (__APPLE__)
    semaphore->sem = dispatch_semaphore_create(0);
    semaphore->count = 0;
#else
    sem_init(semaphore, 0, 0);
#endif
}

void osal_semaphore_wait(osal_semaphore_t *semaphore)
{
#if defined (__APPLE__)
    bool res = (dispatch_semaphore_wait(semaphore->sem, DISPATCH_TIME_FOREVER) == 0);
    if (res)
        semaphore->count--;
#else
    while (sem_wait(semaphore) != 0);
#endif
}

bool osal_semaphore_try_wait(osal_semaphore_t *semaphore) {
#if defined (__APPLE__)
    bool res = (dispatch_semaphore_wait(semaphore->sem, DISPATCH_TIME_NOW) == 0);
    if (res)
        semaphore->count--;
    return res;
#else
    return (sem_trywait(semaphore) == 0);
#endif
}

bool osal_semaphore_timed_wait(osal_semaphore_t *semaphore, uint32_t microseconds) {
#if defined (__APPLE__)
    bool res = (dispatch_semaphore_wait(semaphore->sem, microseconds * 1000uLL) == 0);
    if (res)
        semaphore->count--;
    return res;
#else
    struct timespec timeToWait;
    struct timeval now;
    gettimeofday(&now,NULL);
    timeToWait.tv_nsec = (now.tv_usec + microseconds) * 1000uLL;
    timeToWait.tv_sec = now.tv_sec + timeToWait.tv_nsec / 1000000000uLL;
    timeToWait.tv_nsec %= 1000000000uLL;
    int res;
    while (true) {
        res = sem_timedwait(semaphore, &timeToWait);
        if (res == 0 || errno == ETIMEDOUT)
            break;
    }
    return (res == 0);
#endif
}

int osal_semaphore_get_value(osal_semaphore_t *semaphore) {
#if defined (__APPLE__)
    return semaphore->count;
#else
    int value = 0;
    sem_getvalue(semaphore, &value);
    return value;
#endif
}

void osal_semaphore_post(osal_semaphore_t *semaphore) {
#if defined (__APPLE__)
    semaphore->count++;
    dispatch_semaphore_signal(semaphore->sem);
#else
    sem_post(semaphore);
#endif
}

void osal_semaphore_deinit(osal_semaphore_t *semaphore) {
#if defined (__APPLE__)
    dispatch_release(semaphore->sem);
#else
    sem_close(semaphore);
    sem_destroy(semaphore);
#endif
}
