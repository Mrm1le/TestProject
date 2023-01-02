#include "testthread.h"

int main()
{
    
    auto new_node = std::make_shared<IMDiag>();
    diag_handle = std::make_shared<IMDiag>();
    for (int i = 0; i < 10; ++i)
    {
        // auto new_node = std::make_shared<IMDiag>();
        
        // auto new_node = new IMDiag();
        new_node->start();
        new_node->stop();
        // diag_handle->start();
        // diag_handle->stop();
    }
    
    // auto new_node = std::make_shared<IMDiag>();
    return 0;
}