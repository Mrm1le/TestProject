#ifndef IO_H
#define IO_H

// #include <direct.h>
#include <io.h>
#include <stdlib.h>
#include <time.h> 
#include <cstdio>
#include <dirent.h>
#include <chrono>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/sysinfo.h>
#include <iomanip>

#include <iostream>
#include <string>
#include <fstream>
#include <unistd.h>


static bool mkNewDir(std::string folder_name){
    if(access(folder_name.c_str(), 0) == 0){
        return false;
    }

    if(mkdir(folder_name.c_str(),   0755)==-1)  
    {   
        perror("mkdir   error");   
        return   false;   
    }  

    return true;   // folder exists
}

static int getRandNum(int a = 0, int b = 100){
    srand((unsigned)time(NULL)); 
    if(a<b){
        return rand();
    }
    return rand() % (b-a+1)+ a;
}

static void getFileNames(const std::string& path, std::vector<std::string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))){
        std::cout<<"Folder doesn't Exist!"<<std::endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
            if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
                filenames.push_back(path + "/" + ptr->d_name);
        }
    }
    closedir(pDir);
}

static void removeFile(const std::string& path){
    if(remove(path.c_str())!=0)
    {
        std::cout<<"failed to delete:"<< path <<std::endl;
    }
}

static void removeFolder(const std::string& path){
    std::vector<std::string> filenames;
    getFileNames(path, filenames);
    for(auto f: filenames){
        removeFile(f);
    }
    rmdir(path.c_str());
}

static int getCpuNum(){
    return int(get_nprocs());
}


#endif