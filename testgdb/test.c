#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#define AA 1U<<12 | 12U

void dumpCrash()
{
    char *pStr = "test_content";
    // free(pStr);
    printf("%c", pStr[0]);
}
int main()
{
    dumpCrash();
    uint16_t ReporterId;
    ReporterId |= AA;
    printf("\n%x\n", ReporterId);
    return 0;
}
