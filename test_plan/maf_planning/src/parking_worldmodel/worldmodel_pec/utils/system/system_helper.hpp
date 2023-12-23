#pragma once

#include <cstdio>
#include <dirent.h>
#include <exception>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

namespace putils {

class SystemHelper {
public:
  static bool isPathAbsolute(const std::string &path) {
    if (path.empty()) {
      return false;
    } else {
      return path[0] == '/';
    }
  }
  static bool checkFileOrFolderExist(const std::string &file_name) {
    return (access(file_name.c_str(), F_OK) == 0);
  }

  static bool checkFileOrFolderReadable(const std::string &file_name) {
    return (access(file_name.c_str(), R_OK) == 0);
  }

  static bool deleteFile(const std::string &file_name) {
    return (remove(file_name.c_str()) == 0);
  }

  static bool checkFileOrFolderWritable(const std::string &file_name) {
    return (access(file_name.c_str(), W_OK) == 0);
  }

  static bool createFolder(const std::string &folder_full_path) {
    return (mkdir(folder_full_path.c_str(), 0775) == 0);
  }

  static bool listFilenamesInFolder(const std::string &folder_full_path,
                                    std::vector<std::string> &out_file_list) {
    out_file_list.clear();
    DIR *dir = NULL;
    struct dirent *ptr = NULL;
    dir = opendir(folder_full_path.c_str()); /// open the dir
    if (dir == NULL) {
      return false;
    }
    while ((ptr = readdir(dir)) != NULL) /// read the list of this dir
    {
      // 8 -- file 10--link 4--folder
      if (ptr->d_type == 8) {
        out_file_list.push_back(std::string(ptr->d_name));
      }
    }
    closedir(dir);
    return true;
  }

  static bool
  generateTimestampListInFolder(const std::string &folder_full_path,
                                std::vector<long long> &out_ts_list) {
    std::vector<std::string> file_list;
    bool ls_success = listFilenamesInFolder(folder_full_path, file_list);

    if (!ls_success) {
      return false;
    }

    out_ts_list.clear();
    for (int i = 0; i < (int)file_list.size(); i++) {

      std::string file_name = file_list[i];
      std::string file_name_without_ext = file_name;
      if (file_name.find('.') != std::string::npos) {
        file_name_without_ext = file_name.substr(0, file_name.find('.'));
      }

      long long num = -1;
      try {
        num = std::stoll(file_name_without_ext);
      } catch (std::exception &ex) {

        // printf("num invalid \n");
        continue;
      }
      if (std::to_string(num) == file_name_without_ext) {
        out_ts_list.push_back(num);
      }
    }
    std::sort(out_ts_list.begin(), out_ts_list.end());

    return true;
  }

  static bool getFileSize(const std::string &file_full_path, int &output_size) {
    output_size = -1;
    struct stat statbuf;
    if (stat(file_full_path.c_str(), &statbuf) != 0) {
      return false;
    }
    output_size = statbuf.st_size;
    return true;
  }

  static bool readFileIntoLines(const std::string &file_full_path,
                                std::vector<std::string> &lines) {
    std::ifstream ifs(file_full_path);
    if (!ifs.is_open()) {
      return false;
    }
    lines.clear();
    std::string line;
    while (getline(ifs, line)) {
      lines.push_back(line);
    }
    ifs.close();
    return true;
  }

  static bool readFileIntoString(const std::string &file_full_path,
                                 std::string &str) {
    str.clear();
    std::ifstream ifs(file_full_path);
    if (!ifs.is_open()) {
      return false;
    }
    std::string line;
    while (getline(ifs, line)) {
      str = str + line + "\n";
    }
    ifs.close();
    return true;
  }

  // static bool writeLinesIntoFile(const std::string &file_full_path,
  //                                const std::vector<std::string> &lines) {
  //   std::ofstream ofs(file_full_path);
  //   if (!ofs.is_open()) {
  //     return false;
  //   }

  //   for (std::string line : lines) {
  //     ofs << line << std::endl;
  //   }
  //   ofs.close();

  //   return true;
  // }

  static bool splitFullPathIntoPathAndName(const std::string &file_full_path,
                                           std::string &path,
                                           std::string &name) {

    int split_pos = file_full_path.find_last_of('/');
    if (split_pos < 0) {
      path = "";
      name = file_full_path;
    } else {
      path = file_full_path.substr(0, split_pos + 1);
      name = file_full_path.substr(split_pos, file_full_path.length());
    }
    return true;
  }
};

} // namespace putils
