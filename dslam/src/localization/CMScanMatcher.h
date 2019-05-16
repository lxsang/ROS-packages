#ifndef CMSCANMATCHER_H
#define CMSCANMATCHER_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/Pose2D.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
//#include <pcl/point_types.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>


#include "3rd/graph_slam/graph_slam.h"

#include "utils/luaconf.h"
#include "common.h"

using namespace g2o;

namespace dslam {
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> ScanOdomSyncPolicy;
    class CMScanMatcher {
        public:
            CMScanMatcher();
            ~CMScanMatcher();

            bool optimize(int );

            void configure(Configuration &, ros::NodeHandle &);
            //void getLastKnownPose(geometry_msgs::Pose& pose);
            void publishTf();
            void publishGraph();
        private:
            void scanOdomCallBack(const sensor_msgs::LaserScanConstPtr &, const nav_msgs::OdometryConstPtr &);
            RobotLaser *convertScan(SE2 , const sensor_msgs::LaserScanConstPtr &);            
            
            message_filters::Synchronizer<ScanOdomSyncPolicy>* synchronizer_;
            message_filters::Subscriber<sensor_msgs::LaserScan>* scan_sync_;
            message_filters::Subscriber<nav_msgs::Odometry>* odom_sync_;

            ros::Publisher trajectory_p_;
            ros::Publisher scan_cloud_p_;

            //bool received_odom_;
            bool initialized_;

            std::string base_frame_;
            std::string fixed_frame_;
            std::string scan_topic_;
            std::string odom_topic_;
            std::string odom_frame_;

            double kf_dist_linear_;
            double kf_dist_linear_sq_;
            double kf_dist_angular_;
            double max_laser_range_;
            int max_optimization_step_;

            tf::Transform fixed_to_dom_;
            tf::TransformBroadcaster tf_broadcaster_;
            SE2 laser2base_, current_estimated_;
            SE2 latest_odom_msg_, last_used_odom_msg_;
            GraphSLAM slam_;
    };
}

#endif