#ifndef COMMON_H
#define COMMON_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <stdlib.h>


template <class T1, class T2>
double distance(T1 from, T2 to)
{
    return sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2));
}

bool my_pose(geometry_msgs::Pose *pose, tf::TransformListener *l, std::string global_frame, std::string base_frame)
{
    tf::StampedTransform transform;
    try
    {
        l->lookupTransform(global_frame, base_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        //ROS_ERROR("TF %s - %s: %s", map_frame.c_str(), base_frame.c_str(), ex.what());
        return false;
    }
    pose->position.x = transform.getOrigin().getX();
    pose->position.y = transform.getOrigin().getY();
    pose->position.z = transform.getOrigin().getZ();
    pose->orientation.x = transform.getRotation().getX();
    pose->orientation.y = transform.getRotation().getY();
    pose->orientation.z = transform.getRotation().getZ();
    return true;
}

#endif