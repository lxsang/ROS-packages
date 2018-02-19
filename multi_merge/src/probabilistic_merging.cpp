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
#include <ros/ros.h>
#include "multi_master_bridge/MapData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"

std::string other_map_,my_map_,merged_map_topic,map_update_;
std::map<std::string,  multi_master_bridge::MapData> pipeline;
nav_msgs::OccupancyGrid global_map;
geometry_msgs::Pose mypose;
ros::Publisher  global_map_pub;
ros::Publisher  map_update_pub;
double map_resolution_;
void getRelativePose(geometry_msgs::Pose p, geometry_msgs::Pose q, geometry_msgs::Pose &delta, double resolution) {
  
    delta.position.x = round((p.position.x - q.position.x) / resolution);
    delta.position.y = round((p.position.y - q.position.y) / resolution);
    delta.position.z = round((p.position.z - q.position.z) / resolution);
}


void resolve_mapsize(geometry_msgs::Point theirpose,const nav_msgs::OccupancyGrid msg)
{
    double x,y;
    int ow,oh,w,h;
    geometry_msgs::Point  delta;

    delta.x = mypose.position.x - theirpose.x;
    delta.y = mypose.position.y - theirpose.y;

    // max w and max h
    w = (int)global_map.info.width  ;
    h = (int)global_map.info.height ;
    ow = (int)msg.info.width + round(fabs(delta.x/ msg.info.resolution));
    oh = (int)msg.info.height + round(fabs(delta.y/ msg.info.resolution));
    
    if(delta.x < 0) delta.x = 0.0;
    if(delta.y < 0) delta.y = 0.0;
   
       
    // min x and min y
    x = msg.info.origin.position.x - delta.x ;
    y = msg.info.origin.position.y - delta.y ;

    if(x < global_map.info.origin.position.x)  
    {
        double dx = global_map.info.origin.position.x - x;
        ow =  (int)msg.info.width;
        w += round(dx/ msg.info.resolution);
        global_map.info.origin.position.x = x;   
    }
    if(y <  global_map.info.origin.position.y) {
        double dy =  global_map.info.origin.position.y - y;
        global_map.info.origin.position.y = y;
        oh = (int)msg.info.height;
        h += round(dy/ msg.info.resolution);
        //h = h ;
    }
    
    w = w>ow?w:ow;
    h = oh>h?oh:h;

    global_map.info.width = w ;
    global_map.info.height = h ;
}

void register_local_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("Registering local map");
    
    multi_master_bridge::MapData mdata;
    mdata.position = mypose.position;
    mdata.map = *msg;
    mdata.x = 0.0;
    mdata.y = 0.0;
    resolve_mapsize(mdata.position, *msg);
    map_update_pub.publish(mdata);
    pipeline["localhost"] = mdata;
}
void register_neighbor_map(const multi_master_bridge::MapData::ConstPtr& msg)
{
    ROS_INFO("Registering neighbor map");
    resolve_mapsize(msg->position, msg->map);
    pipeline[msg->ip] = *msg;
}

void mege_pipeline()
{
    // reset global map
    global_map.data.resize(global_map.info.width*global_map.info.height, -1);
    std::fill(global_map.data.begin(), global_map.data.end(), -1);
    // calculate probability for each cell
    double odds = 1.0, oddsi = 0.0;
    geometry_msgs::Pose offset;
    geometry_msgs::Pose delta;
    std::map<std::string,  multi_master_bridge::MapData>::iterator it;
    int di, dj;
    for(int i = 0; i < global_map.info.width; i++)
    {
        for(int j = 0; j < global_map.info.height; j++)
        { 
            odds = 1.0;  
            for ( it = pipeline.begin(); it != pipeline.end(); it++ )
            {
                getRelativePose(global_map.info.origin, it->second.map.info.origin,offset, global_map.info.resolution);
                offset.position.x = abs(offset.position.x);
                offset.position.y = abs(offset.position.y);
                geometry_msgs::Pose q ;
                q.position = it->second.position;
                getRelativePose(mypose,q ,delta, global_map.info.resolution);
                // now calculate the correspond point in the other map
                di = i + delta.position.x - offset.position.x;
                dj = j + delta.position.y - offset.position.y;
                if(di < 0 || dj < 0 || di >= it->second.map.info.width || dj >= it->second.map.info.height)
                    continue;
                // now get the cell
                double cell = (double)it->second.map.data[di + dj*it->second.map.info.width];
                if(cell == -1.0 || cell == 100.0) continue;
                oddsi = cell/ (100.0-cell);

                odds *= oddsi;
            }
            if(odds == 1.0) continue;
            global_map.data[i + j*global_map.info.width] = round((odds/ (1.0 + odds))*100.0);
        }
    }
    ROS_INFO("publish global map");
    ros::Time now = ros::Time::now();
    global_map.info.map_load_time = now;
    global_map.header.stamp = now;
    global_map_pub.publish(global_map);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_merging");
	ros::NodeHandle n("~");

    n.param<std::string>("other_map",other_map_, "/other_map");
    n.param<std::string>("my_map",my_map_, "/map");
    n.param<std::string>("map_update_topic",map_update_, "/map_update");
    n.param<std::string>("merged_map_topic",merged_map_topic, "/global_map");
    n.param<double>("init_z",mypose.position.z, 0.0);
	n.param<double>("init_x",mypose.position.x, 0.0);
	n.param<double>("init_y",mypose.position.y, 0.0);
    n.param<double>("map_resolution",map_resolution_, 0.05);
    /*int w =  round((map_width_m_ )/map_resolution_);
    int h = round((map_height_m_ )/map_resolution_);
    global_map.data.resize(w*h,-1);*/
    global_map.info.width = 0;
    global_map.info.height = 0;
    global_map.info.resolution = map_resolution_;
    global_map.info.origin.position.x = 0.0;
    global_map.info.origin.position.y = 0.0;
    global_map.info.origin.position.z = 0.0;
    global_map.info.origin.orientation.x = 0.0;
    global_map.info.origin.orientation.y = 0.0;
    global_map.info.origin.orientation.z = 0.0;
    global_map.info.origin.orientation.w = 1.0;
    // subscribe to this map
    ROS_INFO("My initial position is [%f,%f,%f]\n",mypose.position.x,mypose.position.y, mypose.position.z);
    ROS_INFO("My Map topic is %s",my_map_.c_str());
    ROS_INFO("Other map topic is %s", other_map_.c_str());
    ROS_INFO("Global map topic is %s", merged_map_topic.c_str());
    ros::Subscriber sub = n.subscribe<multi_master_bridge::MapData>(other_map_, 50,&register_neighbor_map);
	ros::Subscriber sub1 = n.subscribe<nav_msgs::OccupancyGrid>(my_map_, 50,&register_local_map);
    // publisher register
    global_map_pub = n.advertise<nav_msgs::OccupancyGrid>(merged_map_topic, 50, true);
    map_update_pub = n.advertise<multi_master_bridge::MapData>(map_update_, 50, true);
    ros::Rate r(10);
    while(ros::ok())
    {
        ros::spinOnce();
        mege_pipeline();
        r.sleep();
    }
}