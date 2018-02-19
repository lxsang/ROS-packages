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
#include "Int32Helper.h"


int Int32Helper::hash()
{
    return _H("std_msgs/Int32");
}
void Int32Helper::rawToRosMsg(uint8_t* data)
{
    std_msgs::Int32* ptr = new std_msgs::Int32;
    ptr->data = (int)*data;
    _rawsize = sizeof(int);
    _msg = (void*) ptr; 
}
int Int32Helper::rosMsgToRaw(uint8_t** data)
{
    std_msgs::Int32* ptr = (std_msgs::Int32*)_msg;
    *data = (uint8_t*)malloc(sizeof(int));
    memcpy(*data,&(ptr->data),sizeof(int));
    return sizeof(int);
}

