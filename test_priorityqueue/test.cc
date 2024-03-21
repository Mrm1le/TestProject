#include <iostream>
#include <memory>
#include <queue>
#include <vector>

template <typename T> class Compare {
public:
  bool operator()(std::shared_ptr<T> t1, std::shared_ptr<T> t2) {
    return t1->GetID() > t2->GetID();
  }
};
class Test {
public:
  Test(int id) : id(id) {}
  int GetID() { return id; }

private:
  int id;
};
int main() {
  std::priority_queue<std::shared_ptr<Test>, std::vector<std::shared_ptr<Test>>,
                      Compare<Test>>
      pq;
  for (int i = 0; i < 100; ++i) {
    pq.push(std::shared_ptr<Test>(new Test(i)));
  }
  for (int i = 0; i < 100; ++i) {
    printf("id %d\n", pq.top()->GetID());
    pq.pop();
  }
  return 0;
}
