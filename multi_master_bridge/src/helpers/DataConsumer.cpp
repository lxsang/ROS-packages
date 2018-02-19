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
#include "DataConsumer.h"
template <class T1, class T2>  
void Consumer<T1,T2>::consume(ros::Publisher** pub,portal_data_t* pd)
{
    if(!*pub)
    {
        ros::NodeHandle n;
        *pub = new ros::Publisher();
        *(*pub) = n.advertise<T1>(pd->publish_to, 1,true);
    }
    // consume data
    T2 helper;
    helper.consume(pd->data, pd->size);
    helper.template publish<T1>(*pub);
}

void DataConsumer::consume(ros::Publisher** pub,portal_data_t* pd)
{
   
    if(pd->hash == StringHelper::hash())
        INPUT<std_msgs::String,StringHelper>(pub,pd);

    else  if(pd->hash == Int32Helper::hash())
        INPUT<std_msgs::Int32,Int32Helper>(pub,pd);

    else  if(pd->hash == HeaderHelper::hash())
        INPUT<std_msgs::Header,HeaderHelper>(pub,pd);

    else if(pd->hash == PointHelper::hash())
        INPUT<geometry_msgs::Point,PointHelper>(pub,pd);
    
    else if(pd->hash == QuaternionHelper::hash())
        INPUT<geometry_msgs::Quaternion,QuaternionHelper>(pub,pd);

    else if(pd->hash == PoseHelper::hash())
        INPUT<geometry_msgs::Pose,PoseHelper>(pub,pd);

    else if(pd->hash == MapMetaDataHelper::hash())
        INPUT<nav_msgs::MapMetaData,MapMetaDataHelper>(pub,pd);
    
    else if(pd->hash == OccupancyGridHelper::hash())
        INPUT<nav_msgs::OccupancyGrid,OccupancyGridHelper>(pub,pd);
    else if(pd->hash == MapDataHelper::hash())
        INPUT<multi_master_bridge::MapData,MapDataHelper>(pub,pd);
    else
        ROS_INFO("Cannot find data handle for hash %d", pd->hash);
    
}