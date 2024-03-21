#include <iostream>
#include <string>

typedef struct {
  std::string a;
} tmp;

int main() {
  tmp tmp_;
  tmp_.a = 1;
  std::string dump_time_ = "";
  dump_time_ += std::to_string(1) + std::string(".");
  printf("%s\n", dump_time_.c_str());

  // 字符串常用函数
  const char *test = "bcdefg";
  std::string s("C++ Primer");
  std::string s1(test);      // 常用构造方法
  std::string s2(s1, 2);     // 从下标2开始拷贝
  std::string s3(s1, 2, 2);  // 从下标2开始拷贝2个字符
  auto s4 = s1.substr(2, 2); // 取从第2个字符开始的2个字符
  s1.insert(0, "aa");        // 开始处插入zz
  s1.erase(0, 1);            // 删除开始出的一个字符
  s1.append("h");
  s1.replace(6, 1, "hi");  // 将下标为6的字符替换成"hi"
  auto a = s1.find("h");   // 返回第一次出现的下标
  auto b = s2.compare(s2); // 比较字符串
  std::cout << s1 << "/" << s2 << "/" << s3 << "/" << s4 << "/" << a << "/" << b
            << std::endl;

  // 字符串与数字常用转换
  std::string s5 = "pi =" + std::to_string(3.1415926);
  auto pi = std::stod(
      s5.substr(s5.find_first_of("+-.0123456789"))); // 提取字符串中的数字

  std::cout << pi << std::endl;
  // auto s2 = s;
  // s == s2;

  double gx = 0.000002 / 0.2;
  double gy = 0.03 / 0.2;
  double gtheta = 0.05 / 0.2;
  std::string aa = std::to_string(gx) + "_" + std::to_string(gy) + "_" +
                   std::to_string(gtheta);
  std::cout << aa << std::endl;
  return 0;
}