#include <algorithm>
#include <iostream>
#include <list>

int main() {
  std::list<std::string> slst;
  slst.push_back("22");
  slst.push_back("333");
  slst.push_front("1");
  std::list<std::string> slst2;
  slst2.push_back("4444");
  slst2.push_back("4444");
  slst.merge(slst2);
  slst.remove_if([](const std::string &s) {
    return s == "22";
  });             // lambda函数传递remove条件
  slst.reverse(); // 反序排列
  slst.sort([](const std::string &s1, const std::string &s2) {
    return s1.size() > s2.size();
  });            // 按条件排序
  slst.unique(); // 删除重复元素
  std::for_each(slst.cbegin(), slst.cend(),
                [](const std::string &s) { std::cout << s << std::endl; });
  return 0;
}