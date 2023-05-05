#include <vector>
#include <string>
#include <iostream>
#include <numeric>
#include <algorithm>
#include <functional>
#include <unistd.h>

bool IsShorter(const std::string &s1, const std::string &s2)
{
    return s1.size() < s2.size();
}

bool CheckSize(const std::string &s, std::size_t sz)
{
    return s.size() <= sz;
}

int main()
{
    std::vector<std::string> svec;

    svec.emplace_back("000");        // 插入元素的3种常用方法
    svec.insert(svec.begin(), "11"); // push_front()的另类实现
    std::cout << svec.size() << "/" << svec.max_size() << std::endl;
    svec.push_back("2");
    std::cout << "svec capacity " << svec.capacity() << std::endl;
    svec.reserve(100000000); // 一次性分配可容纳十个元素的内存
    std::cout << "svec capacity " << svec.capacity() << std::endl;

    std::vector<std::string> b(20, "nothing");                // 一种构造方式
    b.resize(10, "thing");                                   // 保留旧元素，添加新元素
    std::cout << "b size " << b.size() << std::endl;         // resize()改变元素数量
    std::cout << "b capacity " << b.capacity() << std::endl; // resize()不改变容器容量
    // b.pop_back();
    for (auto i : b) // 普通遍历
    {
        std::cout << i << std::endl;
    }
    b.pop_back(); // 弹出最后一个元素

    auto l = svec.end();
    auto ll = svec.back();
    std::cout << *(--l) << " " << ll << std::endl; // 注意end()和back()的区别

    for (auto iter = svec.cbegin(); iter != svec.end(); ++iter) // 迭代器遍历
    {
        std::cout << iter->data() << std::endl;
        std::cout << *iter << std::endl;
    }

    // std::vector<std::string> svec;
    std::vector<int> ivec;
    std::vector<int> ivec2;
    // std::cout << svec[10] << std::endl; // 数组越界检查
    // std::cout << svec.at(10) << std::endl;

    // 常用泛型算法
    auto id = std::find(svec.cbegin(), svec.cend(), "11");
    printf("id [%s] [0x%x]\n", id->c_str(), id);             // find查找指定元素
    auto num = std::count(svec.cbegin(), svec.cend(), "11"); // count返回个数
    std::string tmp("join svec: ");
    std::string group = std::accumulate(svec.cbegin(), svec.cend(), tmp); // 求和函数
    printf("num [%d]\n", num);                                            // find查找指定元素
    std::cout << "group " << group << std::endl;

    std::fill_n(std::back_inserter(ivec), 3, 0);                                    // 插入元素
    std::replace(ivec.begin(), ivec.end(), 0, 3);                                   // 把所有0替换为3
    std::replace_copy(ivec.cbegin(), ivec.cend(), std::back_inserter(ivec2), 3, 6); // 不改变ivec， 将3替换为6， 放入ivec2
    auto unique = std::unique(ivec2.begin(), ivec2.end());                          // 指向最后
    ivec2.erase(unique, ivec2.end());

    for (auto i : ivec2)
    {
        std::cout << i << " ";
    }

    // 向算法传递函数
    std::sort(svec.begin(), svec.end(), IsShorter); // 自定义比较函数 IsShorter
    for (auto i : svec)
    {
        std::cout << i << " ";
    }

    // 向算法传递lambda函数
    std::sort(svec.begin(), svec.end(), [](const std::string &s1, const std::string &s2)
              { return s1.size() > s2.size(); }); // 自定义lambda比较函数
    for (auto i : svec)
    {
        std::cout << i << " ";
    }

    int biggest_length = 2;
    // for_each 算法
    std::for_each(std::find_if(svec.begin(), svec.end(), [biggest_length](const std::string &s)
                               { return s.size() < biggest_length; }),
                  svec.end(), [](const std::string &s)
                  { std::cout << s << " "; }); // 捕获局部变量biggest_length
    std::cout << std::endl;

    std::cout << "equal to 000 times: "
              << std::count_if(svec.cbegin(), svec.cend(), [](const std::string &s)
                               { return s == "000"; })
              << std::endl;

    // 用bind代替lambd函数, 传入find_if算法
    std::for_each(std::find_if(svec.begin(), svec.end(),
                               std::bind(CheckSize, std::placeholders::_1, biggest_length)),
                  svec.end(), [](const std::string &s)
                  { std::cout << s << " "; }); // 捕获局部变量biggest_length
    std::cout << std::endl;

    std::cout << "end success" << std::endl;

    pause();
    return 0;
}