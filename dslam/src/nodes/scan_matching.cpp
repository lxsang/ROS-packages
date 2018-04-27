#include "localization/ScanMatcher.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
//#include "mapping/Mapping.h"

using namespace dslam;


ScanMatcher* matcher_;
ros::Publisher  local_map_pub,trajectory_p_;
//Mapping map_builder_;
geometry_msgs::Pose pose;
geometry_msgs::PoseArray estimated_poses_;
std::string map_frame, laser_frame,robot_base_frame;
void run();


/*void publish_tranform()
{ 
   while (ros::ok())
    { 
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        matcher_->getTransform(transform);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame, "odom"));
        //matcher_->visualize(particles_p_);
    }
    
}*/

void run()
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        matcher_->getLastKnownPose(pose);
        estimated_poses_.poses.push_back(pose);
        estimated_poses_.header.frame_id = map_frame;
        estimated_poses_.header.stamp = ros::Time::now();
        trajectory_p_.publish(estimated_poses_);
        rate.sleep();
    }
}
/*
void localmapping()
{
    while (ros::ok())
    {
        if (!matcher_->keyframes.empty())
        {
            map_builder_.buildFrom(matcher_->keyframes);
            nav_msgs::OccupancyGrid map = map_builder_.getMap();
            ros::Time now = ros::Time::now();
            map.info.map_load_time = now;
            map.header.stamp = now;
            local_map_pub.publish(map);
        }
    }
}*/
int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "dslam");
    ros::NodeHandle nh_local("~");

    std::string config_file;
    nh_local.param<std::string>("conf_file", config_file, "config.lua");
    Configuration config(config_file);
    Configuration localisation = config.get<Configuration>("localisation", Configuration());
    //Configuration mapping = config.get<Configuration>("mapping", Configuration());
    //Configuration local_map = mapping.get<Configuration>("submap", Configuration());
    //map_builder_.configure(mapping);

    map_frame = localisation.get<std::string>("global_frame", "map");
    laser_frame = localisation.get<std::string>("laser_frame", "scan_frame");
    robot_base_frame = localisation.get<std::string>("robot_base", "base_link");


    //map_topic = config.get<std::string>("map_topic", "/map");
    //scan_subscriber_ = nh_local.subscribe(scan_topic, 1, &laserScanCallback);
    trajectory_p_ = nh_local.advertise<geometry_msgs::PoseArray>("/trajectory", 1);

    double frequency = config.get<double>("frequency", 25);
    ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
    ros::Rate rate(frequency);
    //local_map_pub = nh_local.advertise<nav_msgs::OccupancyGrid>(map_topic, 1, true);
    pose.position.x = 0.0; // - offset_.x();
    pose.position.y = 0.0; // - offset_.y();
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    estimated_poses_.poses.push_back(pose);
    estimated_poses_.header.frame_id = map_frame;
    estimated_poses_.header.stamp = ros::Time::now();

    matcher_ = new ScanMatcher();

    matcher_->configure(localisation, nh_local);

    boost::thread t1(&run);
    //boost::thread t2(&localmapping);
    //boost::thread t3(&publish_tranform);
    //t1.join();

    while (ros::ok())
    {
        ros::spinOnce();
        //publish_tranform();
        //if(estimated_poses_.poses.size() > 0)
        //rate.sleep();
    }
    return 0;
}
