#include <vector>
#include <string>
#include <iostream>
int main()
{
    std::vector<std::string> a;
    std::cout <<  a.size() <<"/" << a.max_size() << std::endl;
    a.emplace_back("no");
    std::cout <<  a.size() <<"/" << a.max_size() << std::endl;
    a.insert(a.begin(), "yes");
    std::cout <<  a.size() <<"/" << a.max_size() << std::endl;
    
    std::vector<std::string> b(10, "nothng");

    b.resize(20, "thing");
    // b.pop_back();
    for(auto i : b) //普通遍历
    {
        std::cout << i << std::endl;
        // printf("%s\n", i.c_str());
        b.pop_back();
    }

    auto l = a.end();
    auto ll = a.back();
    std::cout << *(--l) << " " << ll << std::endl; // 注意end()和back()的区别

    for(auto iter=a.cbegin(); iter!=a.end(); ++iter) // 迭代器遍历
    {
        std::cout << iter->data() << std::endl;
        std::cout << *iter << std::endl;
    }

    std::vector<std::string> svec;
    // std::cout << svec[10] << std::endl; // 数组越界检查
    // std::cout << svec.at(10) << std::endl;

    std::cout << "end success" << std::endl;
    return 0;
}