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
#include "StringHelper.h"


int StringHelper::hash()
{
    return _H("std_msgs/String");
}
void StringHelper::rawToRosMsg(uint8_t* data)
{
    std_msgs::String* ptr = new std_msgs::String;
    char* cs = (char*)malloc(_rawsize+1);
    memcpy(cs,data,_rawsize);
    cs[_rawsize] = '\0';
    ptr->data = string(cs); 
    _msg = (void*) ptr; 
}
int StringHelper::rosMsgToRaw(uint8_t** data)
{
    
    std_msgs::String* ptr = (std_msgs::String*)_msg;
    int len = ptr->data.size();
    *data = (uint8_t*)malloc(len);
    memcpy(*data,ptr->data.c_str(),len);
    return len;
}

