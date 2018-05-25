#include <ros/ros.h>
#include <stdlib.h>
#include <random>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseArray.h>
std::string base_frame;
nav_msgs::Odometry last_odom;
bool init;
double pose[3];
tf::TransformBroadcaster *tf_broadcaster_;
double alpha1 = 0.05;   //degree/degree
double alpha2 = 15.0;     //degree/m
double alpha3 = 0.01;   // m/meter
double alpha4 = 0.0001; // m/degree


//geometry_msgs::PoseArray estimated_poses_;
//ros::Publisher pub_;

double get_noise(double x, double sd)
{
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(x, sd);
    return distribution(generator);
}
void publish_odom_tf(const nav_msgs::Odometry::ConstPtr &odom)
{
    if (!init)
    {
        last_odom = *odom;
        pose[0] = odom->pose.pose.position.x;
        pose[1] = odom->pose.pose.position.y;
        pose[2] = tf::getYaw(odom->pose.pose.orientation);

        init = true;
    }
    else
    {
        // now simulate a motion model
        double dx, dy, rot1, rot2, theta1, theta2, theta_prime, trans;

        dx = odom->pose.pose.position.x - last_odom.pose.pose.position.x;
        dy = odom->pose.pose.position.y - last_odom.pose.pose.position.y;

        trans = sqrt(dx * dx + dy * dy);
        theta1 = tf::getYaw(last_odom.pose.pose.orientation);
        theta2 = tf::getYaw(odom->pose.pose.orientation);

        rot1 = atan2(dy, dx) - theta1;
        rot2 = theta2 - theta1 - rot1;

        double sd_rot1 = alpha1 * fabs(rot1) + alpha2 * trans;
        double sd_rot2 = alpha1 * fabs(rot2) + alpha2 * trans;
        double sd_trans = alpha3 * trans + alpha4 * (fabs(rot1) + fabs(rot2));

        /*
        //publish the point cloud
        estimated_poses_.poses.clear();
        for(int i = 0; i< 1000; i++)
        {
            double t = trans + distribution1(generator1);
            geometry_msgs::Pose mp;
            mp.position.x = pose[0] + t*cos(theta1 + rot1);
            mp.position.y = pose[1] + t*sin(theta1 + rot1);
            mp.position.z = 0.0;
            tf::Quaternion q = tf::createQuaternionFromYaw(theta1 + distribution2(generator2) + distribution3(generator3));
            mp.orientation.x = q.getX();
            mp.orientation.y = q.getY();
            mp.orientation.z = q.getZ();
            mp.orientation.w = q.getW();
            estimated_poses_.poses.push_back(mp);
        }
        estimated_poses_.header.frame_id = "odom";
        estimated_poses_.header.stamp = ros::Time::now();
        pub_.publish(estimated_poses_);
        */
        trans += get_noise(0, sd_trans*sd_trans);
        rot1 += get_noise(0, sd_rot1*sd_rot1);
        rot2 += get_noise(0,sd_rot2*sd_rot2);

        pose[0] += trans * cos(theta1 + rot1);
        pose[1] += trans * sin(theta1 + rot1);
        pose[2] = theta1 + rot1 + rot2;
        last_odom = *odom;
    }

    geometry_msgs::Point p;
    p.x = pose[0]; 
    p.y = pose[1];
    p.z = 0.0;

    // publish transform
    tf::Transform msg(tf::createQuaternionFromYaw(pose[2]), tf::Point(tf::Vector3(p.x, p.y, p.z)));
    tf::StampedTransform transform_msg(msg, odom->header.stamp, "odom", base_frame);
    tf_broadcaster_->sendTransform(transform_msg);
    // get
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "noisy_odometry");
    ros::NodeHandle private_nh("~");

    init = false;

    std::string old_odom_topic;
    private_nh.param<std::string>("old_odom_topic", old_odom_topic, "/odom");
    private_nh.param<std::string>("base_frame", base_frame, "base_footprint");
    private_nh.param<double>("alpha1", alpha1, 0.05);
    private_nh.param<double>("alpha2", alpha2, 15.0);
    private_nh.param<double>("alpha3", alpha3, 0.01);
    private_nh.param<double>("alpha4", alpha4, 0.0001);

    //pub_ = private_nh.advertise<geometry_msgs::PoseArray>("/distribution", 1);

    ros::Subscriber sub_ph = private_nh.subscribe<nav_msgs::Odometry>(old_odom_topic, 1, &publish_odom_tf);

    tf_broadcaster_ = new tf::TransformBroadcaster();
    ros::spin();
}