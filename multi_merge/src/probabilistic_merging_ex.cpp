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

#define UNKNOWN -1
std::map<std::string,  multi_master_bridge::MapData> pipeline;
nav_msgs::OccupancyGrid global_map;
geometry_msgs::Pose mypose;
ros::Publisher  global_map_pub;
void getRelativePose(geometry_msgs::Pose p, geometry_msgs::Pose q, geometry_msgs::Pose &delta, double resolution) {
  
    delta.position.x = round((p.position.x - q.position.x) / resolution);
    delta.position.y = round((p.position.y - q.position.y) / resolution);
    delta.position.z = round((p.position.z - q.position.z) / resolution);
}

void resolve_mapsize(geometry_msgs::Point theirpose,const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    double x,y;
    int ow,oh,w,h;
    geometry_msgs::Point  delta;

    delta.x = mypose.position.x - theirpose.x;
    delta.y = mypose.position.y - theirpose.y;

    // max w and max h
    w = (int)global_map.info.width  ;
    h = (int)global_map.info.height ;
    ow = (int)msg->info.width + round(fabs(delta.x/ msg->info.resolution));
    oh = (int)msg->info.height + round(fabs(delta.y/ msg->info.resolution));
    //ow += msg->info.width - w;
    //oh += msg->info.height - h;
    ROS_INFO("%f %f %d %d > %d %d", round(fabs(delta.x/ msg->info.resolution)), round(fabs(delta.y/ msg->info.resolution)), ow, oh, w,h );
    if(delta.x < 0) delta.x = 0.0;
    if(delta.y < 0) delta.y = 0.0;
   
       
    // min x and min y
    x = msg->info.origin.position.x - delta.x ;
    y = msg->info.origin.position.y - delta.y ;

    if(x < global_map.info.origin.position.x)  
    {
        double dx = global_map.info.origin.position.x - x;
        ow =  (int)msg->info.width;
        w += round(dx/ msg->info.resolution);
        global_map.info.origin.position.x = x;   
    }
    if(y <  global_map.info.origin.position.y) {
        double dy =  global_map.info.origin.position.y - y;
        global_map.info.origin.position.y = y;
        oh = (int)msg->info.height;
        h += round(dy/ msg->info.resolution);
        //h = h ;
    }
    
    w = w>ow?w:ow;
    h = oh>h?oh:h;

   //if(w > global_map.info.width) 
        global_map.info.width = w ;
    //if(h > global_map.info.height) 
        global_map.info.height = h ;

    ROS_INFO("XY (%f %f) map: (%f %f), (%f %f)", global_map.info.origin.position.x,global_map.info.origin.position.y, msg->info.origin.position.x, msg->info.origin.position.y, x, y);
    /*ROS_INFO("WH (%d %d) map: (%d %d)", maxw,maxh, msg->info.width, msg->info.height);*/
}

void register_local_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    multi_master_bridge::MapData mdata;
    mdata.position.x = 0.0;
    mdata.position.y = 0.0;
    mdata.position.z = 0.0;
    mdata.map = *msg;
    resolve_mapsize(mdata.position, msg);
    pipeline["local"] = mdata;
}

void register_local_map_1(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    multi_master_bridge::MapData mdata;
    mdata.position.x = -0.75;
    mdata.position.y = -1.03;
    mdata.position.z = 0.0;
    mdata.map = *msg;
    resolve_mapsize(mdata.position, msg);
    pipeline["local_1"] = mdata;
}
void register_local_map_2(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    multi_master_bridge::MapData mdata;
    mdata.position.x = 0.83;
    mdata.position.y = -1.03;
    mdata.position.z = 0.0;
    mdata.map = *msg;
    resolve_mapsize(mdata.position, msg);
    pipeline["local_2"] = mdata;
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
    double cell;
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
                cell = (double)it->second.map.data[di + dj*it->second.map.info.width];
                if(cell == -1.0 || cell == 100.0) continue;
                oddsi = cell/ (100.0-cell);

                odds *= oddsi;
            }
            if(odds == 1.0) continue;
            global_map.data[i + j*global_map.info.width] = round((odds/ (1.0 + odds))*100.0);
        }
    }
    ROS_INFO("publish global map");
    global_map_pub.publish(global_map);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_merging");
	ros::NodeHandle n("~");

	
    /*int w =  round((60.0 )/0.05);
    int h = round((60.0 )/0.05);
    global_map.data.resize(w*h,-1);*/
    global_map.data.resize(100,-1);
    global_map.info.width = 10;
    global_map.info.height = 10;
    global_map.info.resolution = 0.05;
    global_map.info.origin.position.x = 0.0;
    global_map.info.origin.position.y = 0.0;
    global_map.info.origin.position.z = 0.0;
    global_map.info.origin.orientation.x = 0.0;
    global_map.info.origin.orientation.y = 0.0;
    global_map.info.origin.orientation.z = 0.0;
    global_map.info.origin.orientation.w = 1.0;
    mypose.position.x = -0.73;
    mypose.position.y = -1.03;
    mypose.position.z = 0.0;
    ros::Rate r(1);
    ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid>("/pmap", 50,&register_local_map);
    ros::Subscriber sub1 = n.subscribe<nav_msgs::OccupancyGrid>("/pmap_1", 50,&register_local_map_1);
    ros::Subscriber sub2 = n.subscribe<nav_msgs::OccupancyGrid>("/pmap_2", 50,&register_local_map_2);
    global_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/global_map", 50, true);
    while(ros::ok())
    {
        ros::spinOnce();
        mege_pipeline();
        r.sleep();
    }
}