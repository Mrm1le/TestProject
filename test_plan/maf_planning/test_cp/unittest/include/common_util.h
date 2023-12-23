#pragma once

#include <unistd.h>
#include <string>

inline std::string getCurrentDir(){
    char *path = get_current_dir_name();
    return std::string(path);
}

inline std::string getResourceDir(){
    return getCurrentDir() + "/resources/";
}


