#include <stdio.h>
int main()
{
    char a = 0;
    int b = 10000;
    while (--b > 0){
        a++;
        printf("%d, ", a);
    }
}