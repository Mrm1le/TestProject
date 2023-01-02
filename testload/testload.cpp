#include <iostream>
#include <stdio.h>
#include <unistd.h>

int main()
{
    char buff[1000];
    while (true)
    {
        // 获取cpu负载
        // 前三位分别是 1、5、15分钟内平均的进程数。第四个是正在运行的进程数，分母是进程总数；最后一个是最近运行的进程ID
        // 0.38 0.50 0.51 2/115 16332
        FILE *fd = fopen("/proc/loadavg", "r");
        size_t b = 1000;
        if (fd != NULL)
        {
            fgets(buff, sizeof(buff), fd);
            float a = 0.00;
            // 程序只读取一分钟的平均进程数
            sscanf(buff, "%f", &a);
            // fclose(fd);
            std::cout << buff << std::endl;
        }
        sleep(1);
    }
}