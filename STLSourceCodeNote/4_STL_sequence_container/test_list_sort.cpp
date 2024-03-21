#include <list>

template <class _Tp, class _Alloc>
void sort_list(std::list<_Tp, _Alloc> &mylist) {
  if (mylist.size() > 1) {
    std::list<_Tp, _Alloc> __carry;
    std::list<_Tp, _Alloc> __counter[64];
    int __fill = 0;
    while (!mylist.empty()) {
      __carry.splice(__carry.begin(), *this, begin());
      int __i = 0;
      while (__i < __fill &&
             !__counter[__i].empty()) { // 检查 __carry 是否可以与 __counter
                                        // 数组中的某个非空链表进行合并
        __counter[__i].merge(__carry); //_carry 合并到 __counter[__i] 中
        __carry.swap(
            __counter[__i++]); // 交换它们的内容，以便下一次循环中能继续操作
      }
      __carry.swap(__counter[__i]);
      if (__i == __fill)
        ++__fill;
    }

    for (int __i = 1; __i < __fill; ++__i)
      __counter[__i].merge(__counter[__i - 1]);
    mylist.swap(__counter[__fill - 1]);
  }
}

int main() {}
