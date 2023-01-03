#include <vector>
#include <string>
#include <iostream>
#include <numeric>
#include <algorithm>
int main()
{
    std::vector<std::string> a;

    a.emplace_back("0"); // 插入元素的3种常用方法
    a.insert(a.begin(), "1"); //push_front()的另类实现
    std::cout <<  a.size() <<"/" << a.max_size() << std::endl;
    a.push_back("2");
    std::cout << "a capacity " << a.capacity() << std::endl;
    a.reserve(10); //一次性分配可容纳十个元素的内存
    std::cout << "a capacity " << a.capacity() << std::endl;

    std::vector<std::string> b(20, "nothng"); //一种构造方式
    b.resize(10, "thing"); //保留旧元素，添加新元素
    std::cout << "b size " << b.size() << std::endl; // resize()改变元素数量 
    std::cout << "b capacity " << b.capacity() << std::endl; // resize()不改变容器容量 
    // b.pop_back();
    for(auto i : b) //普通遍历
    {
        std::cout << i << std::endl;
    }
    b.pop_back(); //弹出最后一个元素

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

    // 常用泛型算法
    auto id = std::find(a.cbegin(), a.cend(), "1");
    printf("id [%s] [0x%x]\n", id->c_str(), id); //find查找指定元素
    auto num = std::count(a.cbegin(), a.cend(), "1"); //count返回个数
    std::string tmp("join a: ");
    std::string group = std::accumulate(a.cbegin(), a.cend(), tmp); // 求和函数
    printf("num [%d]\n", num); //find查找指定元素
    std::cout << "group " << group << std::endl;
    return 0;
}