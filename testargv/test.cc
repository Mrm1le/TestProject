#include <iostream>
int main(int argc, char **argv)
{
    for(int i = 0; i<argc; ++i)
    {
        std::cout << (*argv[i]+1) << std::endl;
    }
}