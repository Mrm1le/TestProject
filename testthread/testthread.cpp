#include "testthread.h"

void IMDiag::on_init()
{
}

void IMDiag::on_running()
{
    pthread_setname_np(pthread_self(), "on_running");
    printf("111\n");
}

int IMDiag::start()
{
    on_init();
    node_thread_ = std::make_shared<std::thread>(
        &IMDiag::on_running, this);
    // on_running();
    return 0;
}

void IMDiag::stop()
{
    if (node_thread_->joinable())
        node_thread_->join();
}

