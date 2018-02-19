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
#include "PointHelper.h"


int PointHelper::hash()
{
    return _H("geometry_msgs/Point");
}
void PointHelper::rawToRosMsg(uint8_t* data)
{
    geometry_msgs::Point* ptr = new geometry_msgs::Point;
    memcpy(&ptr->x, data , sizeof(ptr->x));
    memcpy(&ptr->y, data+sizeof(ptr->x) ,sizeof(ptr->y));
    memcpy(&ptr->z, data + 2*sizeof(ptr->x),sizeof(ptr->z));
    _rawsize = 3*sizeof(ptr->x);
    _msg = (void*) ptr; 
}
int PointHelper::rosMsgToRaw(uint8_t** data)
{
    
    geometry_msgs::Point* ptr = (geometry_msgs::Point*)_msg;
    int len=3*sizeof(ptr->x);
    *data = (uint8_t*) malloc(len);
    memcpy(*data,&ptr->x, sizeof(ptr->x));
    memcpy(*data + sizeof(ptr->x),&ptr->y, sizeof(ptr->y));
    memcpy(*data + 2*sizeof(ptr->x),&ptr->z, sizeof(ptr->z));
    return len;
}

