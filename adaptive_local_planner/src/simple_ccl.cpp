/**
Copyright (c) 2017 Xuan Sang LE <xsang.le@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
**/
#include "simple_ccl.h"

SimpleCCL::SimpleCCL()
{
    //max_label = 0;
}
SimpleCCL::~SimpleCCL()
{
    data.clear();
}
void SimpleCCL::setMap(nav_msgs::OccupancyGrid map)
{
    dw = map.info.width;
    dh = map.info.height;
    data.clear();
    data.resize(dw*dh,-1);
    //fill(data.begin(),data.end(),-1);
    this->ccl(&map);
}

void SimpleCCL::ccl(nav_msgs::OccupancyGrid* map)
{
    int i,j,idx, label=0;
    int8_t cell;
    vector<cell_t> neighbors;
    // first pass
    for(i = 0; i < dh;i++)
    {
        for(j = 0; j< dw; j++)
        {
            idx = i*dw + j;
            cell = map->data[idx];
            if(cell  >= th) // object
            {
                neighbors = this->neighbors_of((cell_t){j,i,0,cell} ,map, true);

                if(neighbors.size() == 0) // empty neighbor
                {
                    data[idx] = label;
                    // new label subset
                    subset_t lbs;
                    lbs.parent = label;
                    lbs.rank = 0;
                    lbs.cnt = 0;
                    lbs.mass_x = 0;
                    lbs.mass_y = 0;
                    labels_tree.push_back(lbs);
                    label++;
                    //max_label = label;
                }
                else 
                {
                    int minl = this->min_label_of(neighbors);
                    data[idx] = minl;
                    vector<cell_t>::iterator it;
                    for(it = neighbors.begin(); it != neighbors.end(); it++)
                        this->label_union(minl,it->label);
                }
            }
        }

    }
    // second pass
    for(i = 0; i <  dh;i++)
        for(j =0; j < dw; j++)
        {
            idx = i*dw + j;
            if(data[idx] == -1) continue; 
            int l = this->label_find(data[idx]);
            labels_tree[l].cnt++;
            labels_tree[l].mass_x += j;
            labels_tree[l].mass_y += i;
            data[idx] = l;
            labels.insert(l);
        }
}

int SimpleCCL::min_label_of(vector<cell_t> neighbors)
{
    vector<cell_t>::iterator it;

    int minv =  neighbors.begin()->label;
    for(it = neighbors.begin(); it != neighbors.end(); it++)
        if(it->label != 0 && it->label < minv)
            minv = it->label;

    return minv;
}

vector<cell_t> SimpleCCL::neighbors_of(cell_t curr, nav_msgs::OccupancyGrid* map, bool first)
{
    vector<cell_t> neighbors;
    if(curr.x != 0 && map->data[curr.x - 1 + curr.y*dw]>= th) // x-1, y
        neighbors.push_back((cell_t){curr.x - 1, curr.y, data[curr.x-1 + curr.y*dw],curr.value });

    if(curr.y != 0 && map->data[curr.x  + (curr.y-1)*dw]>= th) // x, y - 1
        neighbors.push_back((cell_t){curr.x , curr.y-1, data[curr.x + (curr.y-1)*dw], curr.value});

    if(curr.x != 0 && curr.y != 0 && map->data[curr.x - 1 + (curr.y-1)*dw]>= th) // x - 1, y - 1
        neighbors.push_back((cell_t){curr.x - 1, curr.y-1, data[curr.x-1 + (curr.y-1)*dw], curr.value });

    if(curr.x != dw - 1 && curr.y != 0 && map->data[curr.x + 1 + (curr.y-1)*dw]>= th) // x+1, y - 1
        neighbors.push_back((cell_t){curr.x + 1, curr.y-1, data[curr.x+1 + (curr.y-1)*dw],curr.value });
    
    if(first) return neighbors;
    // this is for second pass
    if(curr.x != dw - 1 && map->data[curr.x + 1 + (curr.y)*dw]>= th) // x+1 , y
        neighbors.push_back((cell_t){curr.x + 1, curr.y, data[curr.x+1 + curr.y*dw],curr.value });

    if(curr.x !=0 && curr.y != dh - 1 && map->data[curr.x - 1 + (curr.y+1)*dw]>= th) // x - 1, y + 1
        neighbors.push_back((cell_t){curr.x - 1, curr.y+1, data[curr.x-1 + (curr.y+1)*dw],curr.value });

    if( curr.y != dh - 1 && map->data[curr.x  + (curr.y+1)*dw]>= th) // y + 1, x
        neighbors.push_back((cell_t){curr.x , curr.y+1, data[curr.x + (curr.y+1)*dw],curr.value });

    if(curr.x != dw - 1 && curr.y != dh - 1 && map->data[curr.x + 1 + (curr.y+1)*dw]>= th) // x+1 , y +1
        neighbors.push_back((cell_t){curr.x + 1, curr.y+1, data[curr.x+1 + (curr.y+1)*dw],curr.value });
    return neighbors;
}

void SimpleCCL::print()
{
    set<int>::iterator it;
    for(it = labels.begin(); it != labels.end();it++)
        printf("LABEL: %d\n",*it+1);
    int i,j;
    for(i=0;i<dh;i++)
    {
        for(j = 0; j < dw; j++)
        {
           printf("%d ", data[i*dw + j] + 1);
        }
        cout << endl;
    }
}

void SimpleCCL::label_union(int x, int y)
{
    int xroot = this->label_find(x);
    int yroot = this->label_find(y);
 
    // Attach smaller rank tree under root of high rank tree
    // (Union by Rank)
    if (labels_tree[xroot].rank < labels_tree[yroot].rank)
        labels_tree[xroot].parent = yroot;
    else if (labels_tree[xroot].rank > labels_tree[yroot].rank)
        labels_tree[yroot].parent = xroot;
 
    // If ranks are same, then make one as root and increment
    // its rank by one
    else
    {
        labels_tree[yroot].parent = xroot;
        labels_tree[xroot].rank++;
    }
}
int SimpleCCL::label_find(int x)
{
    //printf("\tFind label %d\n", x);
    // find root and make root as parent of x (path compression)
    if (labels_tree[x].parent != x)
        labels_tree[x].parent = this->label_find(labels_tree[x].parent);

    return labels_tree[x].parent;   
}
