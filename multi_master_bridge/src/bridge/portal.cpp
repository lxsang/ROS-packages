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
#include "data_portal.h"

#ifdef __cplusplus
}
#endif
#include <signal.h>
#include "../helpers/DataConsumer.h"
static int sockfd=-1;
static unsigned port = -1;
ros::Publisher pub;
//pthread_mutex_t publishing_mutex;

void callback(struct portal_data_t d)
{
    //pthread_mutex_lock(&publishing_mutex);
    try{
	    ros::Publisher* dpub = NULL;
	    DataConsumer consumer;
	    consumer.consume(&dpub,&d);
	    ROS_INFO("[%s] Publish  %d bytes data to:%s\n",d.from,d.size,d.publish_to);
        if(d.data) free(d.data);
    } catch(const char* msg)
    {
        ROS_INFO("PROBLEM: %s", msg);
    }
    //pthread_mutex_unlock(&publishing_mutex);
}

/*service to return portal address*/
/*bool portal_address(multi_master_bridge::PortalPort::Request &req, multi_master_bridge::PortalPort::Response &res)
{
    res.port = port;
    ROS_INFO("Called with port: %d\n", port );
    return true;
}*/

void sig_handle(int s)
{
	if(sockfd > 0)
		close(sockfd);
    //sockfd = -1;
	ROS_INFO("End listener");
    std_msgs::Int32 msg;
    msg.data = -1;
    pub.publish(msg);
	ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "portal",ros::init_options::NoSigintHandler);
	ros::NodeHandle n;
	pub = n.advertise<std_msgs::Int32>("/portal", 1000);
	ros::Rate loop_rate(10);
    signal(SIGINT, sig_handle);
    
    port = 0;
    sockfd = portal_startup(&port);
	std::stringstream ss;
	ss << "Listen on port"<< port;
	std_msgs::Int32 msg;
    msg.data = (int)port;
	ROS_INFO("%s", ss.str().c_str());
	pub.publish(msg);

	int client_sock;
    pthread_t newthread;
    
    while(ros::ok())
    {
        ros::spinOnce();
		pub.publish(msg);
        client_sock = portal_listen(sockfd);
        if (client_sock == -1)
        {
            continue;
        }
        struct portal_callback_t callback_data;
        callback_data.client = client_sock;
        callback_data.callback = callback;
        if (pthread_create(&newthread , NULL,(void *(*)(void *))portal_checkin, (void *)&callback_data) != 0)
            perror("pthread_create");
        else
        {
            //reclaim the stack data when thread finish
            pthread_detach(newthread) ;
        }
        //portal_checkin(&callback_data);
    }

	return 0;
    
}