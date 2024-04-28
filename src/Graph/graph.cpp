#include <Arduino.h>
#include "graph.h"

std::vector<node*> graph;
node* current_node;
int global_id = 0; //running count of ids


void build(node* src, direction_t direction, bool neighbours[4])
{
    src->visited = 1;
    Serial.print("Neighbours ");
    Serial.print(neighbours[0]);
    Serial.print(neighbours[1]);
    Serial.print(neighbours[2]);
    Serial.println(neighbours[3]);

    //fill neighbours
    for(int i = 0; i < 4; i++)
    {
        if(neighbours[i])
        {
            if(i == back)
            {
                //printf("error, detected node in back\n");
                //exit(1);
                continue;
            }

            if(src->dir[(direction + i) % 4] != NULL) continue;

            node* new_node =get_new_node();
            new_node->dir[(direction + i + 2) % 4] = src;
            new_node->visited = 0;
            new_node->id = ++global_id;

            src->dir[(direction + i) % 4] = new_node;

            Serial.print("Add neighbour at ");
            Serial.print((long long)new_node);
            Serial.print(" ");
            Serial.print((long long)src->dir[(direction + i) % 4]);            ~
            Serial.print(" ");
            Serial.println((direction + i) % 4);
        }
        else
        {
            //src->dir[(direction + i) % 4] = NULL;
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
    
    if(next->dir[back] != NULL)
        if((next->dir[back])->visited == 0) 
            stack.push_front(next->dir[back]);
    if(next->dir[right] != NULL)
        if((next->dir[right])->visited == 0) 
            stack.push_front(next->dir[right]);
    if(next->dir[front] != NULL)
        if((next->dir[front])->visited == 0) 
            stack.push_front(next->dir[front]);
    if(next->dir[left] != NULL)
        if((next->dir[left])->visited == 0) 
            stack.push_front(next->dir[left]);
    
    if(stack.size() == 0) 
        return NULL;
    else
    {
        //stack.front()->visited = 1;
        return stack.front();
    }
}

direction_t path_optimized[12 * 12];
direction_t* gen_next_path(node* src, direction_t dir, node* dest, int* t_size)
{
    printf("path\n");
    bool visited[global_id] = {0};
    std::list<node*> queue;
    node* parent[global_id] = {0}; 

    node* recall[global_id];
    int recall_index = 0;

    queue.push_back(dest);
    printf("push done %d\n", dest->id);
    visited[dest->id] = 1;
    parent[dest->id] = NULL;

    while(!queue.empty())
    {
        node* n = queue.front();
        recall[recall_index++] = n;
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

    Serial.println("got parents");
    Serial.println((src->dir[0])->id);

    direction_t path[12 * 12];
    int dir_index = 0;
    node* cur = src;
    while(cur != dest)
    {
        //Serial.print("cur: ");
        //Serial.println((long long)cur);
        node* p = parent[cur->id];
        //Serial.print("parent ");
        //if(p != NULL) Serial.println((long long)p);
        //else Serial.println("NULL");
        for(int i = 0; i < 4; i++)
        {
                    //Serial.print("d2 ");
            if(p->dir[i] == cur)
            {
                        //Serial.print("d3 ");
                while(dir != ((i + 2) % 4))
                {
                            //Serial.print("d4 ");
                    path[dir_index++] = right;
                    dir = (direction_t)((dir + 1) % 4);
                    Serial.print("dir ");
                    Serial.println(dir);
                }
                
                path[dir_index++] = front;

                cur = p; 
            }
        }
    }

    Serial.println("got path");
    int path_index = 0;
    int count = 0;
    for(int i = 0; i < dir_index; i++)
    {
        direction_t v = path[i];
        if(v == right)
        {
            count++;
        }
        else
        {
            if(count == 3) path_optimized[path_index++] = (left);
            else for(int i = 0; i < count; i++) path_optimized[path_index++] = right;
            count = 0; 
            path_optimized[path_index++] = v;
        }
    }    

    if(count == 3) path_optimized[path_index++] = left;
    else for(int i = 0; i < count; i++) path_optimized[path_index++] = right;
    Serial.println("got path optmized");
    *t_size = path_index;

    return path_optimized;
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
