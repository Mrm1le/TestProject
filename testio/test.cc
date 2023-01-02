#include <iostream>
#include <fstream>
#include <string>
/**
 * @brief: hxkasd
 * @return {*}
 */
/**
 * @brief:
 * @param {int} argc
 * @param {char**} argv
 * @return {*}
 */
int main(int argc, char **argv)
{
    int num;
    const std::string ifile = "test.txt";
    std::ifstream in;
    in.open(ifile);
    // in.getline(num, 1);
    if (!in)
    {
        std::cout << "open error!" << std::endl;
    }
    else
    {
        std::cout << "open success" << std::endl;
    } // while(std::cin >> num)
    // {
    //     std::cout << num <<  std::cin.rdstate() << std::endl;
    // }
    return 0;
}