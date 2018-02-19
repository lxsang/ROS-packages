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
#include "HeaderHelper.h"


int HeaderHelper::hash()
{
    return _H("std_msgs/Header");
}
void HeaderHelper::rawToRosMsg(uint8_t* data)
{
    std_msgs::Header* ptr = new std_msgs::Header;
    //conversion code here
    ptr->seq = (int)*data;
    int flen;

    memcpy(&(ptr->seq),data,sizeof(int));
    memcpy(&(ptr->stamp.sec),data+sizeof(int),sizeof(ptr->stamp.sec));
    memcpy(&(ptr->stamp.nsec),data+2*sizeof(int),sizeof(ptr->stamp.nsec));
    memcpy(&flen,data+3*sizeof(int),sizeof(int));
    _rawsize = 4*sizeof(int) + flen;
    char* fid = (char*)malloc(flen+1);
    memcpy(fid,data+4*sizeof(int),flen);
    fid[flen] = '\0';
    ptr->frame_id = string(fid);
    if(fid) free(fid);
    _msg = (void*) ptr; 
}
int HeaderHelper::rosMsgToRaw(uint8_t** data)
{
    
    std_msgs::Header* ptr = (std_msgs::Header*)_msg;
    int len=0;
    int flen = ptr->frame_id.size();
    //conversion code here
    len =   4*sizeof(int) + flen;
    *data = (uint8_t*) malloc(len);
    //copy raw data to buffer
    memcpy(*data,&(ptr->seq),sizeof(int));
    memcpy(*data+sizeof(int),&(ptr->stamp.sec),sizeof(ptr->stamp.sec));
    memcpy(*data+2*sizeof(int),&(ptr->stamp.nsec),sizeof(ptr->stamp.nsec));
    memcpy(*data+3*sizeof(int),&flen,sizeof(int));
    memcpy(*data+4*sizeof(int),ptr->frame_id.c_str(),flen);
    return len;
}

