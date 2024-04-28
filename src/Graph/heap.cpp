#include "graph.h"

node heap[12 * 12] = {0};
int heap_index = 0;

node* get_new_node()
{
    if(heap_index < 12 * 12) return &heap[heap_index++];
    else return NULL;
}
