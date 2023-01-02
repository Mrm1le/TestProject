#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <assert.h>

std::string read_file(std::string path)
{
  std::shared_ptr<FILE> fp(fopen(path.c_str(), "r"),
                           [](FILE *file)
                           { fclose(file); printf("!!!!!\n"); });
  if (fp == nullptr)
    return "";
  fseek(fp.get(), 0, SEEK_END);
  std::vector<char> content(ftell(fp.get()));
  fseek(fp.get(), 0, SEEK_SET);
  auto read_bytes = fread(content.data(), 1, content.size(), fp.get());
  assert(read_bytes == content.size());
  return std::string(content.begin(), content.end());
}

void MyFunc()
{
  std::shared_ptr<std::string> tmp(new std::string("nico"), [](std::string *tmp)
                                   {printf("wow!!![%s]\n", tmp->c_str());});
}

int main()
{
  std::string input = "plan.txt";
  std::string config = read_file(input);
  MyFunc();
}
