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
#define MLOG ROS_INFO
#ifdef __cplusplus
extern "C"
{
#endif
#include "bridge/data_portal.h"

#ifdef __cplusplus
}
#endif
#include "helpers/helper.h"
#include "multi_master_bridge/NeighbourId.h"
#include "multi_master_bridge/NeighbourList.h"
#include "nav_msgs/OccupancyGrid.h"
#define UNKNOWN -1 //s

ros::Publisher pub;
multi_master_bridge::NeighbourList neighbors;
std::string publish_to,_interface,map_update_,map_global_;
double _init_x,_init_y,_init_z;
nav_msgs::OccupancyGrid* global_map;
static int sockfd=-1;
struct inet_id_ id;

int get_updated_zone(multi_master_bridge::MapData* data)
{
	
	data->position.x = _init_x;
	data->position.y = _init_y;
	data->position.z = _init_z;


    int x = -1, y = -1,x1 = -1, y1 = -1, i,j;
    // find the bounding box for the updated zone
    for(i = 0; i <global_map->info.width;i++)
        for(j = 0; j < global_map->info.height;j++)
        {
            if(global_map->data[i+j* global_map->info.width]  != UNKNOWN )
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
            if(global_map->data[j+i* global_map->info.width] != UNKNOWN)
            {
                y = i;
                goto findw;
            }
        }
    
    findw:
    for(i = global_map->info.width -1; i>=0;i--)
        for(j = 0; j < global_map->info.height;j++)
        {
            if(global_map->data[i+j* global_map->info.width]  != UNKNOWN)
            {
                x1 = i;
                goto findh;
            }
        }
    findh:
     for(i = global_map->info.height - 1; i >= 0;i--)
        for(j = 0; j < global_map->info.width;j++)
        {
            if(global_map->data[j+i* global_map->info.width] != UNKNOWN)
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
     ROS_INFO("update zone (%d,%d) (%d,%d)",x, y,w,h);
    data->map.data.resize(w*h,UNKNOWN);
    // copy the update zone to the update map
    for(i=0; i < w;i++)
        for(j = 0; j < h; j++)
            data->map.data[i+j*w] = global_map->data[ x + i + (j+y)*global_map->info.width ];
    return 1;
}

void neighbors_init(const multi_master_bridge::NeighbourList::ConstPtr& msg)
{
    if(!global_map) return;
    if(msg->size == 0) return;
	// send amap
	MapDataHelper hp;
	
    multi_master_bridge::MapData* data = nullptr;
    struct portal_data_t d;
	//send data to all neighbour
	for(int i = 0; i < neighbors.size;i++ )
	{
		if(neighbors.list[i].status == 1)
		{
			ROS_INFO("Feed init map to %s (%s):%d", neighbors.list[i].ip.c_str(),  neighbors.list[i].name.c_str() , neighbors.list[i].port);
			if(!data)
            {
                data = new multi_master_bridge::MapData();
                if(!get_updated_zone(data))
                    return;
	            hp.consume((void*)data);
	            d = hp.getPortalDataFor(inet_ntoa(id.ip));
	            d.publish_to = (char*)publish_to.c_str();
	            d.hash = MapDataHelper::hash();
            }
            teleport_raw_data( neighbors.list[i].ip.c_str(), neighbors.list[i].port,d);
		}
	}
	//if(d.data) free(d.data);
	if(data) delete data;
}
void got_newmap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if(global_map)
        delete(global_map);
	global_map = new nav_msgs::OccupancyGrid(*msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "remote_map_init");
	ros::NodeHandle n("~");
	n.param<std::string>("publish_to",publish_to, "/other_map");
	n.param<std::string>("network_interface",_interface, "wlan0");
	n.param<std::string>("map_topic",map_update_, "/global_map");
	n.param<double>("init_z",_init_z, 0.0);
	n.param<double>("init_x",_init_x, 0.0);
	n.param<double>("init_y",_init_y, 0.0);
	//pub = n.advertise<nav_msgs::OccupancyGrid>("other_map", 1000);
	ros::Subscriber sub = n.subscribe<multi_master_bridge::NeighbourList>("/new_robot", 50,&neighbors_init);
	ros::Subscriber sub1 = n.subscribe<nav_msgs::OccupancyGrid>(map_update_, 100,&got_newmap);
	ros::Rate loop_rate(10);
	id = read_inet_id(_interface.c_str());
    global_map = nullptr;
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
    

	return 0;
    
}
