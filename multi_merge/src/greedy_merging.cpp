/**
Copyright (c) 2017 Xuan Sang LE <xsang.le@gmail.com>

This code is based on a greedy merging implementation of Zhi Yan 
https://github.com/yzrobot/map_merging

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
std::string other_map_,my_map_,merged_map_topic,map_update_;
bool furious_merge;
nav_msgs::OccupancyGrid global_map;
nav_msgs::OccupancyGridPtr local_map;
geometry_msgs::Pose my_pose;
ros::Publisher  global_map_pub;
ros::Publisher  map_update_pub;
std::map<std::string,  multi_master_bridge::MapData> pipeline;
double map_width_m_, map_height_m_, map_resolution_;
void getRelativePose(geometry_msgs::Pose p, geometry_msgs::Pose q, geometry_msgs::Pose &delta, double resolution) {
  
    delta.position.x = round((p.position.x - q.position.x) / resolution);
    delta.position.y = round((p.position.y - q.position.y) / resolution);
    delta.position.z = round((p.position.z - q.position.z) / resolution);
}


void greedyMerging(int delta_x, int delta_y, int x, int y, const nav_msgs::OccupancyGrid their_map, bool furious) {
  for(int i = 0; i < (int)their_map.info.width; i++) {
    for(int j = 0; j < (int)their_map.info.height; j++) {
        if(i+delta_x + x  >= 0 && i+delta_x + x  < (int)global_map.info.width &&
	        j+delta_y + y  >= 0 && j+delta_y + y  < (int)global_map.info.height) {
            int mycell = i + delta_x + x +(j + delta_y + y)*(int)global_map.info.width;
            int theircell = i + j*(int)their_map.info.width;
            if((int)global_map.data[mycell] == UNKNOWN || ( furious && (int)their_map.data[theircell] != UNKNOWN && (int)global_map.data[mycell] != (int)their_map.data[theircell] ))
            {
                //ROS_INFO("merging...");
            global_map.data[mycell] = their_map.data[theircell];
            }
        }
    }
  }
}
void mege_pipeline(bool furious)
{
    geometry_msgs::Pose delta;
    if ( !local_map ) {
        ROS_INFO("Local map not found, wait for it");
        return;
    }
    // fill global map with local content
    global_map.data.resize(global_map.info.width*global_map.info.height, -1);
    std::fill(global_map.data.begin(), global_map.data.end(), -1);

    std::map<std::string,  multi_master_bridge::MapData>::iterator it;
    geometry_msgs::Pose offset;
    getRelativePose(global_map.info.origin, local_map->info.origin, offset, local_map->info.resolution);
    offset.position.x = abs(offset.position.x);
    offset.position.y = abs(offset.position.y);

    
    for(int i= 0;i < local_map->info.width ; i++)
        for(int j = 0; j < local_map->info.height; j++)
            if(local_map->data[i + j*local_map->info.width] != -1)
                global_map.data[(offset.position.y +  j)*global_map.info.width + offset.position.x + i] = local_map->data[i + j*local_map->info.width];
        

    //global_map.reset(new nav_msgs::OccupancyGrid(*local_map));
    for ( it = pipeline.begin(); it != pipeline.end(); it++ )
    {
        geometry_msgs::Pose p;
        p.position = it->second.position;
        ROS_INFO("mergin map of %s with init pose (%f,%f,%f)",it->first.c_str(), it->second.position.x,it->second.position.y,it->second.position.z);
        ROS_INFO("Get relative position");
        getRelativePose(p,my_pose, delta,global_map.info.resolution);
        ROS_INFO("Get map offset");
        getRelativePose(global_map.info.origin,it->second.map.info.origin,offset,global_map.info.resolution);
        offset.position.x = abs(offset.position.x);
        offset.position.y = abs(offset.position.y);
        ROS_INFO("merging");
        greedyMerging(round(delta.position.x), round(delta.position.y),offset.position.x,offset.position.y, it->second.map,furious);

    }
    ros::Time now = ros::Time::now();
    global_map.info.map_load_time = now;
    global_map.header.stamp = now;
    ROS_INFO("Publishing global map");
    global_map_pub.publish(global_map);
}
/*
int get_visible_zone(multi_master_bridge::MapData* data,const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    int x = -1, y = -1,x1 = -1, y1 = -1, i,j;
    // find the bounding box for the updated zone
    for(i = 0; i <msg->info.width;i++)
        for(j = 0; j < msg->info.height;j++)
        {
            if(msg->data[i+j* msg->info.width]  != UNKNOWN )
            {
                x = i;
                goto findy;
            }
        }
    findy:
    if(x == -1) goto end;
    for(i = 0; i <msg->info.height;i++)
        for(j = 0; j < msg->info.width;j++)
        {
            if(msg->data[j+i* msg->info.width] != UNKNOWN)
            {
                y = i;
                goto findw;
            }
        }
    
    findw:
    for(i = msg->info.width -1; i>=0;i--)
        for(j = 0; j < msg->info.height;j++)
        {
            if(msg->data[i+j* msg->info.width]  != UNKNOWN)
            {
                x1 = i;
                goto findh;
            }
        }
    findh:
     for(i = msg->info.height - 1; i >= 0;i--)
        for(j = 0; j < msg->info.width;j++)
        {
            if(msg->data[j+i* msg->info.width] != UNKNOWN)
            {
                y1 = i;
                goto end;
            }
        }
    end:
    if(x == -1 || y == -1 || x1 == -1 || y1 == -1)
    {
        ROS_INFO("No update found");
        return 0;
    }
    // new map data
    int w = x1 - x;
    int h = y1 - y;
    data->x = x;
    data->y = y;
    data->map.header = msg->header;
    data->map.info = msg->info;
    data->map.info.width = w;
    data->map.info.height = h;
     ROS_INFO("update zone (%d,%d) (%d,%d)",x, y,w,h);
    data->map.data.resize(w*h,UNKNOWN);
    // copy the update zone to the update map
    for(i=0; i < w;i++)
        for(j = 0; j < h; j++)
            data->map.data[i+j*w] = msg->data[ x + i + (j+y)*msg->info.width ];
    return 1;
}
*/
void resolve_mapsize(geometry_msgs::Point theirpose,const nav_msgs::OccupancyGrid msg)
{
    double x,y;
    int ow,oh,w,h;
    geometry_msgs::Point  delta;

    delta.x = my_pose.position.x - theirpose.x;
    delta.y = my_pose.position.y - theirpose.y;

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
    ROS_INFO("Local map found");
    multi_master_bridge::MapData visibledata;
    visibledata.position = my_pose.position;
    visibledata.x = 0;
    visibledata.y = 0;
    visibledata.map = *msg;
    resolve_mapsize(visibledata.position, *msg);
    map_update_pub.publish(visibledata);
    local_map.reset(new nav_msgs::OccupancyGrid(*msg)); 
}

void register_neighbor_map(const multi_master_bridge::MapData::ConstPtr& msg)
{
    ROS_INFO("Registering neighbor map");
    resolve_mapsize(msg->position, msg->map);
    pipeline[msg->ip] = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "greedy_merging");
	ros::NodeHandle n("~");
    
    n.param<std::string>("other_map",other_map_, "/other_map");
    n.param<std::string>("my_map",my_map_, "/map");
    n.param<std::string>("map_update_topic",map_update_, "/map_update");
    n.param<std::string>("merged_map_topic",merged_map_topic, "/global_map");
    n.param<bool>("furious_merge",furious_merge, false);
    n.param<double>("init_z",my_pose.position.z, 0.0);
	n.param<double>("init_x",my_pose.position.x, 0.0);
	n.param<double>("init_y",my_pose.position.y, 0.0);
    n.param<double>("map_resolution",map_resolution_, 0.05);
   
    global_map.info.width = 0;
    global_map.info.height = 0;
    global_map.info.resolution = map_resolution_;
    global_map.info.origin.position.x =0.0;
    global_map.info.origin.position.y = 0.0;
    global_map.info.origin.position.z = 0.0;
    global_map.info.origin.orientation.x = 0.0;
    global_map.info.origin.orientation.y = 0.0;
    global_map.info.origin.orientation.z = 0.0;
    global_map.info.origin.orientation.w = 1.0;
    // subscribe to this map
    ROS_INFO("My initial position is [%f,%f,%f]\n",my_pose.position.x,my_pose.position.y, my_pose.position.z);
    ROS_INFO("My Map topic is %s",my_map_.c_str());
    ROS_INFO("Other map topic is %s", other_map_.c_str());
    ROS_INFO("Global map topic is %s", merged_map_topic.c_str());
    ROS_INFO("Furios merge is  %d", furious_merge );
    ros::Subscriber sub = n.subscribe<multi_master_bridge::MapData>(other_map_, 50,&register_neighbor_map);
	ros::Subscriber sub1 = n.subscribe<nav_msgs::OccupancyGrid>(my_map_, 50,&register_local_map);
    local_map = nullptr;
    // publisher register
    global_map_pub = n.advertise<nav_msgs::OccupancyGrid>(merged_map_topic, 50, true);
    map_update_pub = n.advertise<multi_master_bridge::MapData>(map_update_, 50, true);
    ros::Rate r(1);
    while(ros::ok())
    {
        ros::spinOnce();
        mege_pipeline(furious_merge);
        r.sleep();
    }
}