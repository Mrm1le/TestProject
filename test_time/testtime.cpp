#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
int main(void)
{
    struct tm *newtime;
    char tmpbuf[128];
    time_t lt1;
    struct tm *newtime1;
    char fmt[] = "%Y-%m-%d-%H:%M:%S";
    char buf[] = "2000-01-01-00:00:00";
    time(&lt1);
    newtime = localtime(&lt1);
    // printf("%s\n", tmpbuf);
    strptime(buf, fmt, newtime1);
    std::cout << newtime1->tm_min << std::endl;
    strftime(tmpbuf, 128, "Today is %A, day %d of %B in the year %Y.\n", newtime);
    // printf(tmpbuf);
    std::cout << tmpbuf << std::endl;
    return 0;
}