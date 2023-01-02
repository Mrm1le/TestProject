#include <string>

typedef struct
{
    std::string a;
} tmp;

int main()
{
    tmp tmp_;
    tmp_.a = 1;
    std::string dump_time_ = 0;
    dump_time_ = std::to_string(1) + std::string(".");
    printf("%s\n", dump_time_.c_str());
    return 0;
}