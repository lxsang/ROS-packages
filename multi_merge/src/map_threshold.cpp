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
#include "ros/ros.h"
#include "stdio.h"
#include "math.h"
#include "nav_msgs/OccupancyGrid.h"

std::string map_in_topic_, map_out_topic_;
int th_value_;
ros::Publisher  map_pub;
nav_msgs::OccupancyGrid th_map;
void map_threshold(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    
    th_map.header = msg->header;
    th_map.info = msg->info;
    int w =  msg->info.width;
    int h = msg->info.height;
    th_map.data.resize(w*h,-1);
    std::fill(th_map.data.begin(), th_map.data.end(), -1);
    for(int i= 0;i < w*h ; i++)
            if(msg->data[i] > -1)
            {
                if(msg->data[i] > th_value_)
                    th_map.data[i] = 100;
                else if(msg->data[i] <= 20)
                    th_map.data[i] = 0;
                else  
                    th_map.data[i] = -1;
            } 
            else 
                th_map.data[i] = -1;
       
    map_pub.publish(th_map);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_threshold");
	ros::NodeHandle n("~");
    n.param<int>("th_value",th_value_, 50);
    n.param<std::string>("map_in_topic",map_in_topic_, "/map");
    n.param<std::string>("map_out_topic",map_out_topic_, "/threshold_map");
    
    //ROS_INFO("map size (%d, %d)", w, h);
    
    ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid>(map_in_topic_, 50,&map_threshold);
    map_pub = n.advertise<nav_msgs::OccupancyGrid>(map_out_topic_, 50, true);
    ros::spin();
}