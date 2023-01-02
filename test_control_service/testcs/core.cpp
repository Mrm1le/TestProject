#include "core.h"
T* T::get()
{
    if(core!=nullptr)
        return core;
    else{
        core = new T(0);
        return core;
    }
}

int T::change(int in)
{
    core->a = in;
    return 0;    
}

T* T::core = nullptr;