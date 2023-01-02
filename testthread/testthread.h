#pragma once

#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
class IMDiag
{
private:
    /* data */
    std::shared_ptr<std::thread> node_thread_;
    void on_running();
public:
    IMDiag() { printf("000\n");}
    ~IMDiag() { printf("222\n");};
    void on_init();
    int start();
    void stop();
};
