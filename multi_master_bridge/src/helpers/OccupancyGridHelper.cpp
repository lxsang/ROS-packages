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
#include "OccupancyGridHelper.h"
#include "HeaderHelper.h"
#include "MapMetaDataHelper.h"

int OccupancyGridHelper::hash()
{
    return _H("nav_msgs/OccupancyGrid");
}
void OccupancyGridHelper::rawToRosMsg(uint8_t* data)
{
    nav_msgs::OccupancyGrid* ptr = new nav_msgs::OccupancyGrid;
    HeaderHelper header_h;
    MapMetaDataHelper meta_h;
    header_h.consume(data,-1);
    meta_h.consume(data+header_h.rawsize(),-1);
    ptr->header = *((std_msgs::Header*)(header_h.msg()));
    ptr->info = *((nav_msgs::MapMetaData*)(meta_h.msg()));
    int len,offset;
    offset = header_h.rawsize()+meta_h.rawsize();
    memcpy(&len, data+offset, sizeof(int));
    offset += sizeof(int);
    int i=0;
    ptr->data = std::vector<signed char, std::allocator<signed char> >(data+offset, data + offset + len);
    _rawsize = offset + len;
    _msg = (void*) ptr; 
}
int OccupancyGridHelper::rosMsgToRaw(uint8_t** data)
{
    
    nav_msgs::OccupancyGrid* ptr = (nav_msgs::OccupancyGrid*)_msg;
    int hlen,mlen, len=0;
    HeaderHelper header_h;
    MapMetaDataHelper meta_h;
    header_h.consume((void*)&(ptr->header));
    meta_h.consume((void*)&(ptr->info));
    uint8_t *hraw,*iraw;

    hlen = header_h.raw(&hraw);
    mlen = meta_h.raw(&iraw);
    int v = ptr->data.size();
    len = hlen+mlen+sizeof(int)+v;
    *data = (uint8_t*) malloc(len);
    memcpy(*data,hraw,hlen);
    memcpy(*data+hlen,iraw,mlen);
    memcpy(*data+hlen+mlen,&v,sizeof(int));
    memcpy(*data+hlen+mlen+sizeof(int),(uint8_t*)&(ptr->data[0]),v);
  
    if(hraw) free(hraw);
    if(iraw) free(iraw);
   
    return len ;
}

