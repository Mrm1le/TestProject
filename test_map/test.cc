// #include <map>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
// #include <stdio.h>

std::unordered_map<std::string, std::string> BuildMap(std::ifstream &map_file)
{
    std::unordered_map<std::string, std::string> trans_map;
    std::string key;
    std::string value;
    while (map_file >> key && std::getline(map_file, value))
    {
        if (value.size() > 1)
            trans_map[key] = value.substr(1);
        else{
            std::cerr << "no rule for key" << std::endl;
            std::__throw_runtime_error("no rule for key");
        }
    }
    return trans_map;
}

const std::string &Transform(const std::string &s, std::unordered_map<std::string, std::string> &map_file)
{
    auto index = map_file.find(s);
    if (index != map_file.end())
    {
        return index->second;
    }
    else
        return s;
}

void WordTransform(std::ifstream &map_file, std::ifstream &input)
{
    auto trans_map = BuildMap(map_file);
    std::string text;
    while(std::getline(input, text))
    {
        std::istringstream stream(text);
        std::string word;
        bool firstword = true;
        while(stream >> word)
        {
            if(firstword)
                firstword = false;
            else
                std::cout << " ";
            std::cout << Transform(word, trans_map);
        }
        std::cout << std::endl;
    }
}

int main()
{
    std::ifstream transform, paper;
    transform.open("transform.txt");
    paper.open("paper.txt");
    WordTransform(transform, paper);
    return 0;
}