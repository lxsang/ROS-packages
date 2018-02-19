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

std::string other_map_,my_map_,merged_map_topic,map_update_,map_other_;
nav_msgs::OccupancyGridPtr global_map;
bool greedy_local_merge;
geometry_msgs::Pose my_pose;
ros::Publisher  global_map_pub;
ros::Publisher  other_map_pub;
ros::Publisher  map_update_pub;

void getRelativePose(geometry_msgs::Pose p, geometry_msgs::Pose q, geometry_msgs::Pose &delta, double resolution) {
  
    delta.position.x = round((p.position.x - q.position.x) / resolution);
    delta.position.y = round((p.position.y - q.position.y) / resolution);
    delta.position.z = round((p.position.z - q.position.z) / resolution);
}


/*
 * Algorithm 1 - Greedy Merging
 * yz14simpar
 */
 /*
void greedyMerging(int delta_x, int delta_y, const nav_msgs::OccupancyGrid their_map) {
  int offset_w, offset_h;
  offset_h = ((int)global_map->info.height - (int)their_map.info.height);
  offset_w = ((int)global_map->info.width - (int)their_map.info.width);
  ROS_INFO("global (%d,%d) their (%d,%d)",global_map->info.width, global_map->info.height, their_map.info.width, their_map.info.height);
  ROS_INFO("Offset w %d offset h %d", offset_w, offset_h);
  for(int i = 0; i < (int)global_map->info.width; i++) {
    for(int j = 0; j < (int)global_map->info.height; j++) {
      if(i+delta_x - offset_w >= 0 && i+delta_x - offset_w < (int)their_map.info.width &&
	 j+delta_y - offset_h >= 0 && j+delta_y - offset_h < (int)their_map.info.height) {
	if((int)global_map->data[i+j*(int)global_map->info.width] == UNKNOWN)
    {
        //ROS_INFO("merging...");
	  global_map->data[i+j*(int)global_map->info.width] = their_map.data[i+delta_x - offset_w +(j+delta_y - offset_h)*(int)their_map.info.width];
    }
    }
    }
  }
}*/
void greedyMerging(int delta_x, int delta_y, int x, int y, const nav_msgs::OccupancyGrid their_map, bool local) {
  for(int i = 0; i < (int)their_map.info.width; i++) {
    for(int j = 0; j < (int)their_map.info.height; j++) {
      if(i+delta_x + x  >= 0 && i+delta_x + x  < (int)global_map->info.width &&
	 j+delta_y + y  >= 0 && j+delta_y + y  < (int)global_map->info.height) {
    int mycell = i + delta_x + x +(j + delta_y + y)*(int)global_map->info.width;
    int theircell = i + j*(int)their_map.info.width;
	if((int)global_map->data[mycell] == UNKNOWN || ( local && (int)their_map.data[theircell] != UNKNOWN && (int)global_map->data[mycell] != (int)their_map.data[theircell] ))
    {
        //ROS_INFO("merging...");
	  global_map->data[mycell] = their_map.data[theircell];
    }
    }
    }
  }
}

void merge_map( geometry_msgs::Pose p, int x, int y,  nav_msgs::OccupancyGrid msg, bool local)
{
    if(!global_map)
    {
        ROS_INFO("Global map not found, wait for local map");
        return;
    
    }
    else
    {

        // merge two map here
        geometry_msgs::Pose delta;
        ROS_INFO("Get relative position");
        getRelativePose(p,my_pose, delta,global_map->info.resolution);
        ROS_INFO("merging");
        greedyMerging(round(delta.position.x), round(delta.position.y),x,y, msg,local);
         //global_map->header.frame_id = "map";
        ros::Time now = ros::Time::now();
        global_map->info.map_load_time = now;
        global_map->header.stamp = now;
        ROS_INFO("Publishing global map");
        global_map_pub.publish(*global_map);
    }
}
void merge_local_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{

    ROS_INFO("Merging local map");
    // first find map update
    if(!global_map)
    {
        ROS_INFO("Global map not found, create it");
        /*global_map.reset(new nav_msgs::OccupancyGrid(*msg));
        // publish map update as the entire map
        global_map_pub.publish(*global_map);*/

        global_map.reset(new nav_msgs::OccupancyGrid());
        global_map->header = msg->header;
        global_map->info = msg->info;
        global_map->data.resize(msg->info.width*msg->info.height,UNKNOWN);

       // return;
    }
    // NOTE: The two maps has a same dimension
    int x = -1, y = -1,x1 = -1, y1 = -1, i,j;
    // find the bounding box for the updated zone
    for(i = 0; i <global_map->info.width;i++)
        for(j = 0; j < global_map->info.height;j++)
        {
            int8_t cell = msg->data[i+j*msg->info.width];
            if(global_map->data[i+j* global_map->info.width] != cell && cell != UNKNOWN )
            {
                x = i;
                goto findy;
            }
        }
    findy:
    if(x == -1) goto end;
    for(i = 0; i <global_map->info.height;i++)
        for(j = 0; j < global_map->info.width;j++)
        {
            int8_t cell = msg->data[j+i*msg->info.width];
            if(global_map->data[j+i* global_map->info.width] != cell && cell != UNKNOWN)
            {
                y = i;
                goto findw;
            }
        }
    
    findw:
    for(i = global_map->info.width -1; i>=0;i--)
        for(j = 0; j < global_map->info.height;j++)
        {
            int8_t cell = msg->data[i+j*msg->info.width];
            if(global_map->data[i+j* global_map->info.width] != cell && cell != UNKNOWN)
            {
                x1 = i;
                goto findh;
            }
        }
    findh:
     for(i = global_map->info.height - 1; i >= 0;i--)
        for(j = 0; j < global_map->info.width;j++)
        {
            int8_t cell = msg->data[j+i*msg->info.width];
            if(global_map->data[j+i* global_map->info.width] != cell && cell != UNKNOWN)
            {
                y1 = i;
                goto end;
            }
        }
    end:
    if(x == -1 || y == -1 || x1 == -1 || y1 == -1)
    {
        ROS_INFO("no update found");
        return;
    }
    // create new map data with the update zone
    multi_master_bridge::MapData update;
    update.x = x;
    update.y = y;
    int w = x1 - x;
    int h = y1 - y;
    update.map.header = msg->header;
    update.map.info = msg->info;
    update.map.info.width = w;
    update.map.info.height = h;
    update.position = my_pose.position;
    ROS_INFO("update zone (%d,%d) (%d,%d)",x, y,w,h);
    // new map data
    update.map.data.resize(w*h,UNKNOWN);
    // copy the update zone to the update map
    for(i=0; i < w;i++)
        for(j = 0; j < h; j++)
            update.map.data[i+j*w] = msg->data[ x + i + (j+y)*msg->info.width ];
    // publish the update
    map_update_pub.publish(update);
    // merge go here
    ROS_INFO("greedy_local_merge: %d",(int)greedy_local_merge);
    merge_map(my_pose,update.x, update.y, update.map, greedy_local_merge);
    ROS_INFO("Merged");
}
void merge_their_map(const multi_master_bridge::MapData::ConstPtr& msg)
{
    geometry_msgs::Pose p;
    p.position = msg->position;
    ROS_INFO("Merging map from another robot: %s", msg->ip.c_str());
    ROS_INFO("Their init pose is [%f,%f,%f]\n",p.position.x,p.position.y, p.position.z);
    ROS_INFO("update zone (%d,%d) (%d,%d)",msg->x, msg->y,msg->map.info.width,msg->map.info.height);
    other_map_pub.publish(msg->map);
    merge_map(p,msg->x, msg->y,msg->map, false);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "greedy_merging");
	ros::NodeHandle n("~");
    n.param<std::string>("other_map",other_map_, "/other_map");
    n.param<std::string>("my_map",my_map_, "/map");
    n.param<std::string>("map_update_topic",map_update_, "/map_update");
    n.param<std::string>("map_other_topic",map_other_, "/other_map");
    n.param<std::string>("merged_map_topic",merged_map_topic, "/global_map");
    n.param<bool>("greedy_local_merge",greedy_local_merge, true);
    n.param<double>("init_z",my_pose.position.z, 0.0);
	n.param<double>("init_x",my_pose.position.x, 0.0);
	n.param<double>("init_y",my_pose.position.y, 0.0);
    // subscribe to this map
    ROS_INFO("My initial position is [%f,%f,%f]\n",my_pose.position.x,my_pose.position.y, my_pose.position.z);
    ROS_INFO("My Map topic is %s",my_map_.c_str());
    ROS_INFO("Other map topic is %s", other_map_.c_str());
    ROS_INFO("Global map topic is %s", merged_map_topic.c_str());
    ros::Subscriber sub = n.subscribe<multi_master_bridge::MapData>(other_map_, 50,&merge_their_map);
	ros::Subscriber sub1 = n.subscribe<nav_msgs::OccupancyGrid>(my_map_, 50,&merge_local_map);
    global_map = nullptr;
    // publisher register
    global_map_pub = n.advertise<nav_msgs::OccupancyGrid>(merged_map_topic, 50, true);
    other_map_pub = n.advertise<nav_msgs::OccupancyGrid>(map_other_, 50, true);
    map_update_pub = n.advertise<multi_master_bridge::MapData>(map_update_, 50, true);
    ros::Rate r(5);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}