#include <algorithm>
#include <chrono>
#include <iostream>
#include <list>
#include <vector>

class B {
  int a;
};
class A {
public:
  A(B b) : _b(b) {}

private:
  B _b;
};

A getB() {
  B b;
  return b;
}

int main() {
  std::list<int> stl_list;
  constexpr int val = 100000;
  auto start_time = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < val; ++i) {
    stl_list.emplace_front(i);
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time);
  start_time = std::chrono::high_resolution_clock::now();
  start_time = std::chrono::high_resolution_clock::now();
  start_time = std::chrono::high_resolution_clock::now();
  std::cout << "emplace cost: " << duration.count() << " us" << std::endl;
  std::list<int>::iterator iter =
      std::find(stl_list.begin(), stl_list.end(), val);
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                                   start_time);
  start_time = std::chrono::high_resolution_clock::now();
  std::cout << "find cost: " << duration.count() << " us" << std::endl;
  stl_list.reverse();
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                                   start_time);
  std::cout << "reverse cost: " << duration.count() << " us" << std::endl;
  start_time = std::chrono::high_resolution_clock::now();
  stl_list.sort();
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                                   start_time);
  std::cout << "sort cost: " << duration.count() << " us" << std::endl;

  if (iter == stl_list.end())
    std::cout << "mot find " << val << std::endl;
  else
    std::cout << "find " << *iter << std::endl;

  A a = getB();
  return 0;
}