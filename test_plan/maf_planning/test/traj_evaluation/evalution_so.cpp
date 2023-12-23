#include "evalution.h"

extern "C"{
    Eva eva;

    double evaTraj(const char* file_path){
        return eva.evaTraj(file_path);
    }
}