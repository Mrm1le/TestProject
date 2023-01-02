#include "pie_node.h"
#include "mfr_node.h"
int main()
{
    mfr_node* newnode1 = new mfr_node;
    pie_node* newnode2 = new pie_node;
    newnode1->register_data(5);
    newnode2->TxProcess();
    return 0;
}