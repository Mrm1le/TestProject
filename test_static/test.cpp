#include <stdio.h>

int main(){
    int i = 0;
    while(true){
    static int x = i;
    printf("%d %d\n", x, i);
    ++i;
    } 
}