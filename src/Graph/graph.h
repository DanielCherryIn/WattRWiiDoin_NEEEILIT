#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>
#include <list>
#include <string>

typedef enum
{
    front,
    right,
    back,
    left,
} direction_t;

typedef struct node_t{
    struct node_t* dir[4];

    int pos_x;
    int pos_y;

    bool visited;
    int id;

} node;

extern int global_id;
extern node* cur_node;
extern std::vector<node*> graph;
extern std::list<node*> stack;

void build(node* src, direction_t dir, bool neighbours[4]);

node* explore(); //returns destination node

std::vector<direction_t> gen_next_path(node* src, direction_t dir, node* dest);
std::string gen_graph_string(node* src);

#endif