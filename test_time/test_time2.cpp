#include <ctime>
#include <chrono>
#include <iostream>
int main()
{
    auto tmp = std::chrono::system_clock::now();
    auto tt=std::chrono::system_clock::to_time_t(tmp);
    std::string t=ctime(&tt);
    std::cout<<"\ntoday is "<<t;
    std::cout<< "tmp " <<  tmp.time_since_epoch().count() << std::endl;
}