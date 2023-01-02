#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <regex>
#include "time.h"
// #include "absl/strings/match.h"
// #include "absl/strings/str_cat.h"
// #include "absl/strings/str_split.h"

void Stringsplit(const std::string& str, const std::string& split, std::vector<std::string>& res)
{
    //std::regex ws_re("\\s+"); // 正则表达式,匹配空格 
    std::regex reg(split);        // 匹配split
    std::sregex_token_iterator pos(str.begin(), str.end(), reg, -1);
    decltype(pos) end;              // 自动推导类型 
    for (; pos != end; ++pos)
    {
        if(pos->str() != ""){
            res.push_back(pos->str());
            // std::cout << pos->str() ;
        }
    }
}

std::vector<std::string> GetStatsLines(const std::string &stat_file,
                                       const int line_count)
{
    std::vector<std::string> stats_lines;
    std::ifstream buffer(stat_file);
    for (int line_num = 0; line_num < line_count; ++line_num)
    {
        std::string line;
        std::getline(buffer, line);
        if (line.empty())
        {
            break;
        }
        stats_lines.push_back(line);
    }
    return stats_lines;
}

float GetSystemCPUUsage()
{
    const std::string system_cpu_stat_file = "/proc/stat";

    const int users = 1, system = 3, total = 7;
    constexpr static int kSystemCpuInfo = 0;
    static uint64_t prev_jiffies = 0, prev_work_jiffies = 0;
    const auto stat_lines =
        GetStatsLines(system_cpu_stat_file, kSystemCpuInfo + 1);
    std::cout << stat_lines[0] << std::endl;
    if (stat_lines.size() <= kSystemCpuInfo)
    {
        std::cout << "failed to load contents from " << system_cpu_stat_file << std::endl;
        return 0.f;
    }


    //jiffies_stats contains CPU info vector
    std::vector<std::string> jiffies_stats;
        Stringsplit(stat_lines[kSystemCpuInfo], " ", jiffies_stats);

    // const std::vector<std::string> jiffies_stats =
    //     absl::StrSplit(stat_lines[kSystemCpuInfo], ' ', absl::SkipWhitespace());

    if (jiffies_stats.size() <= total)
    {
        std::cout << "failed to get system CPU info from " << system_cpu_stat_file;
        return 0.f;
    }
    uint64_t jiffies = 0, work_jiffies = 0;
    for (int cur_stat = users; cur_stat <= total; ++cur_stat)
    {
        const auto cur_stat_value = std::stoll(jiffies_stats[cur_stat]);
        jiffies += cur_stat_value;
        if (cur_stat <= system)
        {
            work_jiffies += cur_stat_value;
        }
    }
    const uint64_t tmp_prev_jiffies = prev_jiffies;
    const uint64_t tmp_prev_work_jiffies = prev_work_jiffies;
    prev_jiffies = jiffies;
    prev_work_jiffies = work_jiffies;
    if (tmp_prev_jiffies == 0)
    {
        std::cout << "failed to get system CPU info from " << system_cpu_stat_file;
        return 0.f;
    }
    return 100.f * (static_cast<float>(work_jiffies - tmp_prev_work_jiffies) /
                    (jiffies - tmp_prev_jiffies));
}

int main()
{
    int i = 0;
    while( i < 10)
    {
        auto a = GetSystemCPUUsage();
        std::cout << a << std::endl;
        sleep(5);
        ++i;
    }
    return 0;
}