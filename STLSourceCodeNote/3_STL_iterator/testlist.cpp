#include "tianlist.h"
#include <chrono>
#include <ctime>
#include <limits>
int main() {
  list<int> mylist{};
  constexpr int val = 100000;
  auto start_time = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < val; ++i) {
    mylist.insertBack(i);
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time);
  std::cout << "程序运行时间: " << duration.count() << " 毫秒" << std::endl;
  // mylist.display();
  listIter<listItem<int>> begin(mylist.begin());
  listIter<listItem<int>> iter, end;
  iter = find(begin, end, val);
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                                   start_time);
  std::cout << "程序运行时间: " << duration.count() << " 毫秒" << std::endl;
  if (iter == end)
    std::cout << "mot find " << val << std::endl;
  else
    std::cout << "find " << iter->getValue() << std::endl;
  return 0;
}