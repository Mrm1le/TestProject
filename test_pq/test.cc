#include <iostream>
#include <queue>
#include <vector>

int main()
{
    std::vector<int> v;
    v.reserve(100); // 预分配 100 个元素的空间
    std::priority_queue<int, std::vector<int>, std::less<int>> pq(std::less<int>(), std::move(v));
    // 向 pq 中添加元素
    for (int i = 0; i < 100; ++i)
    {
        pq.push(i);
    }
    std::cout << pq.size() << std::endl; // 输出 100
    return 0;
}
