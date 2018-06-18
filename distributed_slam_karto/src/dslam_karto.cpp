
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"
#include "ros/console.h"
#include "dslam.h"

message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
std::string base_frame_, fixed_frame_;
ros::Publisher constraint_pub_;
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

void graphCallback(const distributed_slam_karto::Graph::ConstPtr& msg )
{
    visualization_msgs::Marker points, line_list;

    points.header.frame_id =  line_list.header.frame_id = fixed_frame_;
    points.header.stamp =  line_list.header.stamp = ros::Time::now();
    points.ns =  line_list.ns = "points_and_lines";
    points.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w  = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_list.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.04;
    points.scale.y = 0.04;

    line_list.scale.x = 0.01;
    // Points are green
    points.color.b = 1.0f;
    points.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    geometry_msgs::Point p;
    // vertex

    for ( auto mit = msg->vertices.begin(); mit != msg->vertices.end(); mit++ )
    {

        p.x = mit->corrected_pose.x;
        p.y =  mit->corrected_pose.y;
        p.z = 0;
        points.points.push_back(p);
    }
    
    karto::Vertex<karto::LocalizedRangeScan>* v1, *v2;
    for(auto it = msg->edges.begin(); it != msg->edges.end(); it++)
    {
        v1 = slam->getVertex(it->source_id);
        if(!v1)
        {
             ROS_WARN("Cannot find vertex %d", it->source_id);
             continue;
        }
        v2 = slam->getVertex(it->target_id);
        if(!v2)
        {
             ROS_WARN("Cannot find vertex %d", it->target_id);
             continue;
        }
        p.x = v1->GetObject()->GetCorrectedPose().GetX();
        p.y = v1->GetObject()->GetCorrectedPose().GetY();
        p.z = 0;
        line_list.points.push_back(p);

        
        p.x = v2->GetObject()->GetCorrectedPose().GetX();
        p.y = v2->GetObject()->GetCorrectedPose().GetY();
        p.z = 0;
        line_list.points.push_back(p);
    }

    // publish the message
    constraint_pub_.publish(points);
    constraint_pub_.publish(line_list);
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
    fixed_frame_ = config.get<std::string>("fixed_frame", "map");
    tf::TransformListener tf;
    slam = new DSlamKarto();
    slam->configure(config, nh_local);
    slam->setTFListener(&tf);

    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_local, scan_topic, 5);
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf, odom_frame, 5);
    scan_filter_->registerCallback(boost::bind(&laserCallback, _1));

    std::string constraints_topic = config.get<std::string>("constraints_topic", "/constraints");
    std::string graph_topic = config.get<std::string>("graph_topic", "/graph");

    constraint_pub_ = nh_local.advertise< visualization_msgs::Marker>(constraints_topic, 10);
    //ros::Rate rate(25);
    boost::thread t1(&publishTf);
    bool publish_graph = config.get<bool>("publish_graph",false);
    ros::Subscriber sub_ph;
    if(publish_graph)
    {
        sub_ph = nh_local.subscribe<distributed_slam_karto::Graph>(graph_topic, 5, &graphCallback);
        boost::thread t2(&publishGraph);
    }
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
