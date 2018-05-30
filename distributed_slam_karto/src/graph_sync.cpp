#include "ros/ros.h"
#include "ros/console.h"
#include <distributed_slam_karto/Graph.h>
#include <distributed_slam_karto/SyncGraph.h>

/*
This node subscribe to 
the graph topic of a robot
and sync it to another robot
using the provided service
*/
ros::ServiceClient client;

void graphCallback(const distributed_slam_karto::Graph::ConstPtr& msg )
{
    // call the sync service
  distributed_slam_karto::SyncGraph srv;
  srv.request.graph = *msg;
  if (client.call(srv))
  {
    ROS_INFO("Graph is sync");
  }
  else
  {
    ROS_ERROR("Failed to call service to sync the graph");
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_sync");
    // execute config
    ros::NodeHandle nh_local("~");

    // subscriber to topic
    std::string graph_topic, syn_service;
    nh_local.param<std::string>("graph_topic",graph_topic, "/pose_graph");
    nh_local.param<std::string>("syn_service",syn_service, "/dslam_sync_graph");
    
    ros::Subscriber sub_ph = nh_local.subscribe<distributed_slam_karto::Graph>(graph_topic, 5, &graphCallback);
    client = nh_local.serviceClient<distributed_slam_karto::SyncGraph>(syn_service);
    //t1.join();
    ros::Rate rate(0.5);
    while (ros::ok())
    {
        ros::spinOnce();
        //publish_tranform();
        //if(estimated_poses_.poses.size() > 0)
        rate.sleep();
    }

    return 0;
}
