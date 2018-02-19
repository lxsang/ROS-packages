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
#include "watchdog.h"

#ifdef __cplusplus
}
#endif

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
ros::Publisher pub; 
int broadcast_to,broadcast_rate;
std::string broadcast_interface;
int port = -1;
void callback(const std_msgs::Int32::ConstPtr& msg)
{
	port = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "beacon");
	ros::NodeHandle n("~");
	n.param<int>("broadcast_to", broadcast_to, 9196);
    n.param<std::string>("broadcast_interface",broadcast_interface, "wlan0");
	n.param<int>("refresh_rate",broadcast_rate, 10);
	pub = n.advertise<std_msgs::String>("beacon", 1000);
	ros::Subscriber sub = n.subscribe<std_msgs::Int32>("/portal", 50,
                                                   &callback);
	port = -1;
	ros::Rate loop_rate(broadcast_rate);
	while(ros::ok())
	{
		ros::spinOnce();
		if(port == -1) continue;
		if(send_beacon(broadcast_to,broadcast_interface.c_str(),port))
		{
			std_msgs::String str;
			std::stringstream ss;
			ss << "Sent out beacon "<< port;
			ss << " on "<< broadcast_to;
			str.data = ss.str();
			//ROS_INFO("%s", str.data.c_str());
			pub.publish(str);
		}
		loop_rate.sleep();
	}
	return 0;
    
}