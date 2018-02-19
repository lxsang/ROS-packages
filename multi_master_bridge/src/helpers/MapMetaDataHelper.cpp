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
#include "MapMetaDataHelper.h"
#include "PoseHelper.h"

int MapMetaDataHelper::hash()
{
    return _H("nav_msgs/MapMetaData");
}
void MapMetaDataHelper::rawToRosMsg(uint8_t* data)
{
    nav_msgs::MapMetaData* ptr = new nav_msgs::MapMetaData;
    PoseHelper pose_h;
    pose_h.consume(data+5*sizeof(int),-1);
    ptr->origin = *((geometry_msgs::Pose*)(pose_h.msg()));

    memcpy(&(ptr->map_load_time.sec),data, sizeof(int));
    memcpy(&(ptr->map_load_time.nsec),data+sizeof(int), sizeof(int));
    memcpy(&(ptr->resolution),data+2*sizeof(int), sizeof(int));
    memcpy(&(ptr->width),data+3*sizeof(int), sizeof(int));
    memcpy(&(ptr->height), data+4*sizeof(int), sizeof(int));
    _rawsize = pose_h.rawsize() + 5*sizeof(int);
    _msg = (void*) ptr; 
}
int MapMetaDataHelper::rosMsgToRaw(uint8_t** data)
{
    
    nav_msgs::MapMetaData* ptr = (nav_msgs::MapMetaData*)_msg;
    int len= 0,plen=0;
    PoseHelper pose_h;
    pose_h.consume((void*)&(ptr->origin));
    uint8_t* pose_raw;
    plen = pose_h.raw(&pose_raw);
    len = 5*sizeof(int)+plen;
    *data = (uint8_t*) malloc(len);
    
    memcpy(*data,&(ptr->map_load_time.sec), sizeof(int));
    memcpy(*data+sizeof(int),&(ptr->map_load_time.nsec), sizeof(int));
    memcpy(*data +2*sizeof(int),&(ptr->resolution), sizeof(int));
    memcpy(*data+3*sizeof(int),&(ptr->width), sizeof(int));
    memcpy(*data+4*sizeof(int),&(ptr->height), sizeof(int));
    memcpy(*data+5*sizeof(int),pose_raw, plen);
    if(pose_raw) free(pose_raw);
    return len;
}

