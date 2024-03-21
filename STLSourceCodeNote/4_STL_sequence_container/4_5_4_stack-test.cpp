// file: 4stack-test.cpp

#include <algorithm>
#include <iostream>
#include <list>
#include <stack>

using namespace std;

int main() {
  stack<int, list<int>> istack;
  istack.push(1);
  istack.push(3);
  istack.push(5);
  istack.push(7);

  cout << istack.size() << endl;
  cout << istack.top() << endl;

  istack.pop();
  cout << istack.top() << endl;
  istack.pop();
  cout << istack.top() << endl;
  istack.pop();
  cout << istack.top() << endl;
  cout << istack.size() << endl;
}