#include "pie_node.h"
#include <stdio.h>
int pie_node::TxProcess()
{
    same = T::get();
    printf("t = %d\n",same->a);
    return 0;
}