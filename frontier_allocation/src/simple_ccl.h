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
#ifndef SIMPLE_CCL_H
#define SIMPLE_CCL_H

//#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;


typedef struct{
    int x;
    int y;
    int label;
    int value;
} cell_t;

typedef struct 
{
    int parent;
    int rank;
    int cnt;
    int mass_x;
    int mass_y;
}subset_t;

class SimpleCCL {
public:
    SimpleCCL();
    ~SimpleCCL();
    void setMap(nav_msgs::OccupancyGrid map);
    void print();
    int dw, dh;
    vector<int>data;
    set<int> labels;
    vector<subset_t> labels_tree;
    int8_t th;
private:
    void ccl(nav_msgs::OccupancyGrid* map);
    vector<cell_t> neighbors_of(cell_t curr, nav_msgs::OccupancyGrid* map, bool);
    int label_find(int);
    void label_union(int, int);
    int min_label_of(vector<cell_t>);
    //int max_label;

};

#endif