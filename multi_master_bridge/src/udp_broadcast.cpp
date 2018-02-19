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
#include "bridge/watchdog.h"

#ifdef __cplusplus
}
#endif
#include "helpers/helper.h"

ros::Publisher pub;

void send_newmap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	ROS_INFO("Map available");
	
    OccupancyGridHelper hp;
    
    nav_msgs::OccupancyGrid* m = new nav_msgs::OccupancyGrid(*msg);
    hp.consume((void*)m);
    struct portal_data_t d = hp.getPortalDataFor("");
    d.publish_to = "/other_map";
    d.hash = OccupancyGridHelper::hash();
    ROS_INFO("broadcasting the map");
    upd_data_broadcast(9196,"wlp2s0",d);
    // free allocated memory
    if(d.data) free(d.data);
    //ROS_INFO("Feed x:%f, st.sec:%d, st.nsec:%d, fid:%s", m.seq, m.stamp.sec,m.stamp.nsec,m.frame_id.c_str());
    //pub.publish(m);
}

static int sockfd=-1;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "feed");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe<nav_msgs::OccupancyGrid>("/map", 100,&send_newmap);
    ros::spin();
	return 0;
    
}
