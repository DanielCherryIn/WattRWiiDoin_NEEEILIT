#include "graph.h"

std::vector<node*> graph;
node* current_node;
int global_id = 0; //running count of ids


void build(node* src, direction_t direction, bool neighbours[4])
{
    src->visited = 1;

    //fill neighbours
    for(int i = 0; i < 4; i++)
    {
        if(neighbours[i])
        {
            if(i == back)
            {
                printf("error, detected node in back\n");
                exit(1);
            }

            node* new_node = (node*)malloc(sizeof(node));
            new_node->dir[(direction + i + 2) % 4] = src;
            new_node->visited = 0;
            new_node->id = global_id++;

            src->dir[(direction + i) % 4] = new_node;
        }
        else
        {
            src->dir[(direction + i) % 4] = NULL;
        }
    }
}

std::list<node*> stack;
//single step dfs
node* explore()
{
    if(stack.size() == 0)
        return NULL;

    node* next = stack.front();
    stack.pop_front();

    next->visited = 1;

    if(next->dir[back] != NULL && next->dir[back]->visited == 0) 
        stack.push_front(next->dir[back]);
    if(next->dir[right] != NULL && next->dir[right]->visited == 0) 
        stack.push_front(next->dir[right]);
    if(next->dir[front] != NULL && next->dir[front]->visited == 0) 
        stack.push_front(next->dir[front]);
    if(next->dir[left] != NULL && next->dir[left]->visited == 0) 
        stack.push_front(next->dir[left]);

    if(stack.size() == 0) 
        return NULL;
    else
    {
        //stack.front()->visited = 1;
        return stack.front();
    }
}


std::vector<direction_t> gen_next_path(node* src, direction_t dir, node* dest)
{
    printf("path\n");
    std::vector<bool> visited(global_id, 0);
    std::list<node*> queue;
    std::vector<node*> parent(global_id); 

    queue.push_back(dest);
    printf("push done %d\n", dest->id);
    visited[dest->id] = 1;
    parent[dest->id] = NULL;

    while(!queue.empty())
    {
        node* n = queue.front();
        //printf("use node: %d\n", n->id);
        queue.pop_front();

        for(int i = 0; i < 4; i++)
        {
            if(n->dir[i] != NULL)
            {
                int id = (n->dir[i])->id;
                if(!visited[id])
                {
                    visited[id] = 1;
                    queue.push_back(n->dir[i]);
                    parent[id] = n;
                }
            }
        }
    }

    std::vector<direction_t> path;
    node* cur = src;
    while(cur != dest)
    {
        node* p = parent[cur->id];
        for(int i = 0; i < 4; i++)
        {
            if(p->dir[i] == cur)
            {
                while(dir != ((i + 2) % 4))
                {
                    path.push_back(right);
                    dir = (direction_t)((dir + 1) % 4);
                }
                
                path.push_back(front);

                cur = p; 
            }
        }
    }

    return path;
}

std::string gen_graph_string(node* src)
{
    std::string s = std::string("");
    std::vector<bool> visited(global_id, 0);
    visited[src->id] = 1;

    s += std::to_string(global_id) + ";";

    std::list<node*> queue;
    queue.push_back(src);

    while(!queue.empty())
    {
        node* n = queue.front();
        //printf("use node: %d\n", n->id);
        queue.pop_front();

        for(int i = 0; i < 4; i++)
        {
            if(n->dir[i] != NULL)
            {
                int id = (n->dir[i])->id;
                if(id > n->id)
                { 
                    std::string dir;
                    switch(i)
                    {
                        case front:
                            dir = "f";
                        break;
                        case back:
                            dir = "b";
                        break;
                        case left:
                            dir = "l";
                        break;
                        case right:
                            dir = "r";
                        break;
                    }
                    s += std::to_string(n->id) + " " + std::to_string((n->dir[i])->id) + " " + dir + ";";
                }
                if(!visited[id])
                {
                    visited[id] = 1;
                    queue.push_back(n->dir[i]);
                }
            }
        }
    } 

    return s + "\n\0";
} 