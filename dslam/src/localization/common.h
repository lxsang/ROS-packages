#ifndef COMMOM_H
#define COMMOM_H
#include <Eigen/Dense>
#include <sensor_msgs/LaserScan.h>
#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <angles/angles.h>
typedef struct{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d position;
} landmark_t;
typedef struct{
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;
} dslam_tf_t;
typedef struct{
    dslam_tf_t tf;  // transformation from last key frame
    dslam_tf_t diff;// diff between current keyframe and odom
    dslam_tf_t base2laser;
    sensor_msgs::LaserScan scan;
    int index;
} key_frame_t;
#endif