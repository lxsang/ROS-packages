
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
#include "PoseHelper.h"
#include "PointHelper.h"
#include "QuaternionHelper.h"

int PoseHelper::hash()
{
    return _H("geometry_msgs/Pose");
}
void PoseHelper::rawToRosMsg(uint8_t* data)
{
    geometry_msgs::Pose* ptr = new geometry_msgs::Pose;
    // composite type, use child helpers to consume and produce data
    PointHelper point_h;
    QuaternionHelper quat_h;
    point_h.consume(data,-1);
    quat_h.consume(data+ 3*sizeof(ptr->position.x),-1);
    ptr->position = *((geometry_msgs::Point*)point_h.msg());
    ptr->orientation = *((geometry_msgs::Quaternion*)quat_h.msg());
    _rawsize = point_h.rawsize() + quat_h.rawsize();
    _msg = (void*) ptr; 
}
int PoseHelper::rosMsgToRaw(uint8_t** data)
{
    // composite type, use other helpers to consume and to produce data
    geometry_msgs::Pose* ptr = (geometry_msgs::Pose*)_msg;
    PointHelper point_h;
    QuaternionHelper quat_h;
    point_h.consume((void*)&(ptr->position));
    quat_h.consume((void*)&(ptr->orientation));
    //get raw data
    uint8_t *point_raw, *quat_raw;
    int plen=0,qlen=0;
    plen = point_h.raw(&point_raw);
    qlen = quat_h.raw(&quat_raw);
    int len = plen + qlen;
    *data = (uint8_t*)malloc(len);
    memcpy(*data,point_raw,plen);
    memcpy(*data + plen,quat_raw,qlen);
    if(point_raw) free(point_raw);
    if(quat_raw) free(quat_raw);
    return len;
}

