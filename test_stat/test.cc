#include <sys/stat.h>
#include <string>
#include <unistd.h>

int main()
{
    struct stat sb;
    const std::string config("test.json");
    while (true)
    {
        stat(config.c_str(), &sb);
        printf("modify time %d\n", static_cast<int>(sb.st_mtime));
        sleep(1);
    }
}