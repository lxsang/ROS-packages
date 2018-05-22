#include "ros/ros.h"
#include "ros/console.h"
#include "dslam.h"

using namespace dslam;
DSlamKarto* slam = NULL;
void publishTf()
{
    ros::Rate rate(25);
    while (ros::ok())
    {
        if(!slam) continue;
        slam->publishTf();
        rate.sleep();
    }
}

void publishGraph()
{
    ros::Rate rate(5);
    while (ros::ok())
    {
        if(!slam) continue;
        slam->publishGraph();
        rate.sleep();
    }
}

void publishMap()
{
    ros::Rate rate(0.5);
    while (ros::ok())
    {
        if(!slam) continue;
        slam->publishMap();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    /*if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }*/
    ros::init(argc, argv, "dslam_karto");
    // execute config
    ros::NodeHandle nh_local("~");

    std::string config_file, prefix;
    nh_local.param<std::string>("conf_file", config_file, "config.lua");
    nh_local.param<std::string>("prefix", prefix, "");
    Configuration robot;
    robot.get<std::string>("prefix", prefix);
    // add robot configuration here
    Configuration config(config_file, robot);

    slam = new DSlamKarto();
    slam->configure(config, nh_local);

    //ros::Rate rate(25);
    boost::thread t1(&publishTf);
    bool publish_graph = config.get<bool>("publish_graph",false);
    if(publish_graph)
        boost::thread t2(&publishGraph);
    boost::thread t3(&publishMap);
    //t1.join();
    ros::spin();
    /*while (ros::ok())
    {
        ros::spinOnce();
        //publish_tranform();
        //if(estimated_poses_.poses.size() > 0)
        //rate.sleep();
    }*/

    return 0;
}
