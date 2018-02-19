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
#include "bridge/watchdog.h"

#ifdef __cplusplus
}
#endif
 #include <signal.h>
#include "helpers/helper.h"
#include "helpers/DataConsumer.h"
static int sockfd = -1;

void sig_handle(int s)
{
	if(sockfd > 0)
		close(sockfd);
	ROS_INFO("End sniffing");
	ros::shutdown();
	exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "watch_dog",ros::init_options::NoSigintHandler);
	signal(SIGINT, sig_handle);
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	sockfd = bind_udp_socket(9196);
	struct inet_id_ id = read_inet_id("wlp2s0");
    ros::Publisher* dpub = NULL;
	if(sockfd > 0)
	{
		while (ros::ok())
		{
			ROS_INFO("Wait for data");
			struct portal_data_t d = udp_portal_checkin(sockfd,id);
			if(d.status)
			{
	            DataConsumer consumer;
	            consumer.consume(&dpub,&d);
	            ROS_INFO("[%s] Publish  %d bytes data to:%s\n",d.from,d.size,d.publish_to);
			}
			//ros::spinOnce();
			//loop_rate.sleep();
		}
	}
	return 0;
    
}