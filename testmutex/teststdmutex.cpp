#include <iostream>
// #include <lock_guard>
#include <mutex>
#include <thread>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>
// using namespace std;
std::mutex mutex;
int count[100] = {0};

std::shared_ptr<std::thread> threads[20000];
void Counter() 
{
   //lock_guard 在构造函数里加锁，在析构函数里解锁。
   std::lock_guard<std::mutex> lock(mutex);
 
   int i = ++count[0];
   // std::cout << "count == " << i << std::endl;
}

void Counter1() 
{
   //lock_guard 在构造函数里加锁，在析构函数里解锁。
//    std::lock_guard<std::mutex> lock(mutex);
 
   int i = ++count[0];
   std::cout << "count == " << i << std::endl;
}

int main() 
{
    struct timeval time1;
    struct timeval time2;
    gettimeofday(&time1, NULL);
    auto starttime = time1.tv_sec*1000000 + time1.tv_usec; //suppose run 1ms
   // for (int i = 0; i < 20000; ++i) 
   // {
   //      threads[i] = std::make_shared<std::thread>(
   //                          &Counter);
   // }
   for(auto& th:threads){
      th= std::make_shared<std::thread>(&Counter);
      if(th->joinable())
         th->join();
   }

   gettimeofday(&time2, NULL);
   auto endtime = time2.tv_sec*1000000 + time2.tv_usec; //suppose run 1ms4
    sleep(1);
    printf("--------------------%d\n", endtime - starttime);

    gettimeofday(&time1, NULL);
    starttime = time1.tv_sec*1000000 + time1.tv_usec; //suppose run 1ms
   for(auto& th:threads){
      th= std::make_shared<std::thread>(&Counter1);
      // th->detach();
      if(th->joinable())
         th->join();
   }
   gettimeofday(&time2, NULL);
   endtime = time2.tv_sec*1000000 + time2.tv_usec; //suppose run 1ms4

   printf("-----------------%d\n", endtime - starttime);
   return 0;
}