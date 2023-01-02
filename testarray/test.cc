#include <iostream>
int fool(int *begin, int* end)
{
    while (begin != end)
    {
        std::cout << *begin++ << std::endl;
    }
    return 0;
}

// int fool(int &arr[10])
// {
//     for (auto i : arr)
//     {
//         std::cout << i << std::endl;
//     }
//     return 0;
// }

int main(){
    int a[20] = {0};
    fool(a, &a[19]);
    return 0;
}