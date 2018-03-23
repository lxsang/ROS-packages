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
#ifdef __cplusplus
#define MLOG ROS_INFO
extern "C"
{
#endif
#include "watchdog.h"

#ifdef __cplusplus
}
#endif
 #include <signal.h>
#include "multi_master_bridge/NeighbourId.h"
#include "multi_master_bridge/NeighbourList.h"
#include "std_msgs/Int32.h"
static int sockfd = -1;
int robot_decay_time_s;
int port = -1;
void callback(const std_msgs::Int32::ConstPtr& msg)
{
	port = msg->data;
	//ROS_INFO("Port is %d\n", port);
}
void sig_handle(int s)
{
	if(sockfd > 0)
		close(sockfd);
	ROS_INFO("End sniffing");
	ros::shutdown();
}
int listen_to;
std::string listen_interface;
multi_master_bridge::NeighbourList mylist;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "watch_dog",ros::init_options::NoSigintHandler);
	ros::NodeHandle n("~");
	n.param<int>("listen_to", listen_to, 9191);
    n.param<std::string>("listen_interface",listen_interface, "wlan0");
	n.param<int>("robot_decay_time_s",robot_decay_time_s, 5);
	signal(SIGINT, sig_handle);
	mylist.size = 0;
	ros::Publisher dog_pub = n.advertise<multi_master_bridge::NeighbourList>("/new_robot", 100, true);
	ros::Rate loop_rate(10);

	sockfd = bind_udp_socket(listen_to);
	struct inet_id_ id = read_inet_id(listen_interface.c_str());
	ros::Subscriber sub = n.subscribe<std_msgs::Int32>("/portal", 50,
                                                   &callback);
	bool robot_found = false;
	int found_index = -1;
	if(sockfd > 0)
	{
		while (ros::ok())
		{
			ros::spinOnce();
			if(port == -1)
			{
				continue;
			}
			id.port = port;
			multi_master_bridge::NeighbourId msg;
			struct beacon_t a = sniff_beacon(sockfd,id);
			if(a.status)
			{
				// check if the id is exist
				msg.header.frame_id = std::string(a.hostname);
				msg.ip = std::string(inet_ntoa(a.ip));
				msg.name = std::string(a.hostname);
				msg.port = a.port;
				robot_found = true;
				//mylist.push_back(msg);
			}
			ros::Time t = ros::Time::now();
			for(int i = 0; i < mylist.size;i++)
			{
				int interval = t.sec -  mylist.list[i].header.stamp.sec;
				if(found_index == -1 && robot_found && msg.ip == mylist.list[i].ip && msg.port == mylist.list[i].port)
				{
					found_index = i;
				} else if (mylist.list[i].status != -1 && interval > robot_decay_time_s)
				{
					mylist.list[i].status = -1; // offline
					ROS_INFO(" %s become offline (%d), it is no longer available for exchanging (%d<%d)", mylist.list[i].name.c_str(),mylist.list[i].status,robot_decay_time_s, interval);
				}
			}
			if(robot_found)
			{
				if(found_index != -1)
				{
					//ROS_INFO("neighbour is existing, update its info");
					msg.header.stamp = ros::Time::now();
					mylist.list[found_index].header = msg.header;
					mylist.list[found_index].name = msg.name;
					mylist.list[found_index].port = msg.port;
					if(mylist.list[found_index].status == -1)
						mylist.list[found_index].status = 1;
					else 
						mylist.list[found_index].status = 0;
					found_index = -1;
				} else 
				{
					ROS_INFO("Found neighbour %s at %s:%d", msg.name.c_str(), msg.ip.c_str(), msg.port );
					msg.header.stamp = ros::Time::now();
					mylist.size++;
					msg.status = 1;
					mylist.list.push_back(msg);
				}
				robot_found = false;
			}
			dog_pub.publish(mylist);
			loop_rate.sleep();
		}
	}
	return 0;
    
}