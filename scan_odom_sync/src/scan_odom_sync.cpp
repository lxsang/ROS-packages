#include "ros/ros.h"
//#include "tf2_msgs/TFMessage.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <scan_odom_sync/ScanOdomSync.h>
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> ScanOdomSyncPolicy;
message_filters::Synchronizer<ScanOdomSyncPolicy>* synchronizer_;
message_filters::Subscriber<sensor_msgs::LaserScan>* scan_sync_;
message_filters::Subscriber<nav_msgs::Odometry>* odom_sync_;
ros::Publisher pub;

std::string odom_topic_, scan_topic_, sync_topic_;

void publish_data(const sensor_msgs::LaserScanConstPtr &scan_msg, const nav_msgs::OdometryConstPtr &odom_msg)
{
    scan_odom_sync::ScanOdomSync data;
    data.odom =  *odom_msg;
    data.scan = *scan_msg;
    pub.publish(data);
}

int main(int argc, char **argv)
{
    // TODO: have a problem in 32bit time out of range
    ros::init(argc, argv, "map_exchange");
	ros::NodeHandle nh("~");
    nh.param<std::string>("odom_topic",odom_topic_, "/odom");
	nh.param<std::string>("scan_topic",scan_topic_, "/scan");
    nh.param<std::string>("sync_topic",sync_topic_, "/scan_odom_sync");
    scan_sync_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, scan_topic_, 1);
    odom_sync_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, odom_topic_, 1);
    synchronizer_ = new message_filters::Synchronizer<ScanOdomSyncPolicy>(ScanOdomSyncPolicy(5), *scan_sync_, *odom_sync_);
    synchronizer_->registerCallback(boost::bind(&publish_data, _1, _2));
    pub = nh.advertise<scan_odom_sync::ScanOdomSync>(sync_topic_, 5);
    ros::spin();
}