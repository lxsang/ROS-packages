
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include "ros/ros.h"
#include "ros/console.h"
#include "dslam.h"

message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
std::string base_frame_;
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

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    AnnotatedScan aScan;
    aScan.base_frame_id = base_frame_;
    aScan.scan = scan;
    slam->laserCallback(aScan);
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

    // subscriber to topic
    std::string scan_topic = config.get<std::string>("scan_topic", "/base_scan");
    std::string odom_frame = config.get<std::string>("odom_frame", "odom");
    base_frame_ = config.get<std::string>("base_frame", "base_link");
    tf::TransformListener tf;
    slam = new DSlamKarto();
    slam->configure(config, nh_local);
    slam->setTFListener(&tf);

    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_local, scan_topic, 5);
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf, odom_frame, 5);
    scan_filter_->registerCallback(boost::bind(&laserCallback, _1));

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
