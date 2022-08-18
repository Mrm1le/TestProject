#include <iostream>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// using namespace boost;
boost::mutex mutex;
int count = 0;
 
void Counter() 
{
   //lock_guard 在构造函数里加锁，在析构函数里解锁。
   boost::lock_guard<boost::mutex> lock(mutex);
 
   int i = ++count;
   std::cout << "count == " << i << std::endl;
}
 
int main() 
{
   boost::thread_group threads;
   for (int i = 0; i < 4; ++i) 
   {
      threads.create_thread(&Counter);
   }
 
   threads.join_all();
   return 0;
}