#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#define MEM_SIZE ((size_t)15 * 1024 * 1024 * 1024) // 64GB

int main()
{
    int fd = open("/dev/zero", O_RDWR); // 打开 /dev/zero 文件
    if (fd < 0) {
        perror("open");
        exit(1);
    }
    void*addr[100];

    for (int i = 0; i < 100; i++) {
        addr[i] = mmap(NULL, MEM_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0); // 映射内存
        if (addr[i] == MAP_FAILED) {
            perror("mmap");
            exit(1);
        }
    }
    pause();
    return 0;
}
