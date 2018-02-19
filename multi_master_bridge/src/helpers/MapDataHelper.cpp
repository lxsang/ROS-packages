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
#include "MapDataHelper.h"
#include "PointHelper.h"
#include "OccupancyGridHelper.h"

int MapDataHelper::hash()
{
    return _H("multi_master_bridge/MapData");
}
void MapDataHelper::rawToRosMsg(uint8_t* data)
{
    multi_master_bridge::MapData* ptr = new multi_master_bridge::MapData;
    //conversion code here 
    PointHelper ph;
    OccupancyGridHelper op;
    ph.consume(data,-1);
    int slen;
   
    memcpy(&slen, data  + ph.rawsize(), sizeof(int));
    char* ip = (char*) malloc(slen + 1);
    memcpy(ip, data +  ph.rawsize() + sizeof(int), slen );
    ip[slen] = '\0';
    ptr->ip = string(ip);
    memcpy(&ptr->x,data + ph.rawsize() + slen + sizeof(int), sizeof(int));
    memcpy(&ptr->y, data + ph.rawsize() + slen + 2*sizeof(int), sizeof(int));
    op.consume(data+ ph.rawsize() + slen  + 3*sizeof(int),-1);
    ptr->position = *((geometry_msgs::Point*)ph.msg());
    ptr->map = *((nav_msgs::OccupancyGrid*)op.msg());
    _rawsize = ph.rawsize() + op.rawsize() + slen + 3*sizeof(int);
    _msg = (void*) ptr; 
}
int MapDataHelper::rosMsgToRaw(uint8_t** data)
{
    
    multi_master_bridge::MapData* ptr = (multi_master_bridge::MapData*)_msg;
    PointHelper ph;
    OccupancyGridHelper oh;
    ph.consume((void*)&(ptr->position));
    oh.consume((void*)&(ptr->map));

    uint8_t* praw,*oraw;
    int plen = ph.raw(&praw);
    int olen = oh.raw(&oraw);
    int slen = ptr->ip.size();
   // ROS_INFO("Length %d %d\n",plen,olen);
    *data = (uint8_t*) malloc(plen+olen + slen + 3*sizeof(int));
    memcpy(*data,praw,plen);
    memcpy(*data + plen, &slen,sizeof(int));
    memcpy(*data + plen + sizeof(int),ptr->ip.c_str(), slen);
    memcpy(*data + plen +  slen + sizeof(int), &ptr->x,sizeof(int));
    memcpy(*data + plen +  slen + 2*sizeof(int), &ptr->y, sizeof(int));
    memcpy(*data + plen +  slen + 3*sizeof(int),oraw,olen);
    //conversion code here
    return plen + olen + slen + 3*sizeof(int);
}

