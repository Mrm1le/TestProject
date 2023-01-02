#include <dlfcn.h>
#include <errno.h>
#include <malloc.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <pthread.h>

timer_t g_timer = NULL;
#define TIMEOUT_DEFAULT_VALUE 1 * 6 /* 1分钟 */

static int timer_setup();
static int timer_destroy();
static int timer_run(unsigned int timeout);

/* 格式化时间字符串格式为：Www Mmm dd hh:mm:ss yyyy */
void ctime_get_time(char *string)
{
    time_t t;
    time(&t);
    printf("%s --%s\n", string, ctime(&t)); /* 打印的是当前时区的时间 */
}

static void timeout_handler(int signal_number)
{
    ctime_get_time("timeout_handler");
    timer_destroy();
}

/* 创建定时器 */
static int timer_setup()
{
    int retval;
    timer_t timer;
    struct sigaction act;
    struct sigevent evp;

    memset(&act, 0, sizeof(act));
    act.sa_handler = timeout_handler; /* 倒计时结束的时候调用的函数 */
    act.sa_flags = 0;
    sigemptyset(&act.sa_mask);
    if (sigaction(SIGUSR2, &act, NULL) == -1) {
        return -1;
    }

    memset(&evp, 0, sizeof(evp));
    evp.sigev_value.sival_ptr = &timer;
    evp.sigev_notify = SIGEV_SIGNAL;
    evp.sigev_signo = SIGUSR2;
    retval = timer_create(CLOCK_REALTIME, &evp, &timer);
    if (retval != 0) {
        return -2;
    }

    g_timer = timer;
    ctime_get_time("timer_setup");
    return 0;
}

/* 销毁定时器 */
static int timer_destroy()
{
    int retval = 0;
    if (g_timer != NULL) {
        retval = timer_delete(g_timer);
        g_timer = NULL;
    }

    return retval;
}

/* 启动定时器 */
static int timer_run(unsigned int seconds)
{
    int retval;
    struct itimerspec ts;

    if (g_timer == NULL) {
        timer_setup();
    }

    ts.it_value.tv_sec = seconds; /* 倒计时的时间 */
    ts.it_value.tv_nsec = 0;
    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = 0;
    retval = timer_settime(g_timer, 0, &ts, NULL);
    if (retval != 0) {
        return -1;
    }

    return 0;
}

static void *thread_time(void *data)
{
    printf("thread_time enter\n");
    timer_setup();
    timer_run(TIMEOUT_DEFAULT_VALUE);
}

int main(int argc, char const *argv[])
{
    pthread_t pid;
    pthread_create(&pid, NULL, thread_time, NULL);
    pthread_join(pid, NULL);

    while(1);
    return 0;
}