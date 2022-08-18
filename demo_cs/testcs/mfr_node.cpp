
#include "mfr_node.h"

int mfr_node::register_data(int a)
{
    same =  T::get();
    same->change(a);
    return 0;
}