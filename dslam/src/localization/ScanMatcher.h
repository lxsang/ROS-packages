/*
this code is based on the version of
https://github.com/ccny-ros-pkg/scan_tools/tree/indigo/laser_scan_matcher

*/
#ifndef SCAN_MATCHER_H
#define SCAN_MATCHER_H

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//#include <pcl/point_types.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min
#undef max

#include "utils/luaconf.h"
#include "common.h"


namespace dslam {
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> ScanOdomSyncPolicy;
    class ScanMatcher {
        public:
            ScanMatcher();
            ~ ScanMatcher();
            void configure(Configuration &, ros::NodeHandle &);
            void getLastKnownPose(geometry_msgs::Pose& pose);
            std::list<kf_t> keyframes;
        private:

            ros::Subscriber scan_subscriber_;
            ros::Subscriber odom_subscriber_;
            ros::Subscriber imu_subscriber_;
            message_filters::Synchronizer<ScanOdomSyncPolicy>* synchronizer_;
            message_filters::Subscriber<sensor_msgs::LaserScan>* scan_sync_;
            message_filters::Subscriber<nav_msgs::Odometry>* odom_sync_;

            tf::TransformListener    tf_listener_;
            tf::TransformBroadcaster tf_broadcaster_;

            tf::Transform base_to_laser_; // static, cached
            tf::Transform laser_to_base_; // static, cached, calculated from base_to_laser_


            // **** parameters

            std::string base_frame_;
            std::string fixed_frame_;
            std::string scan_topic_;
            std::string imu_topic_;
            std::string odom_topic_;
            bool publish_tf_;
            std::vector<double> position_covariance_;
            std::vector<double> orientation_covariance_;

            double kf_dist_linear_;
            double kf_dist_linear_sq_;
            double kf_dist_angular_;
            double confidence_factor_;
            // **** What predictions are available to speed up the ICP?
            // 1) imu - [theta] from imu yaw angle - /imu topic
            // 2) odom - [x, y, theta] from wheel odometry - /odom topic
            // If more than one is enabled, priority is imu > odom 

            bool use_imu_;
            bool use_odom_;

            // **** state variables

            boost::mutex mutex_;

            bool initialized_;
            bool received_imu_;
            bool received_odom_;

            tf::Transform f2b_;    // fixed-to-base tf (pose of base frame in fixed frame)
            tf::Transform f2b_kf_; // pose of the last keyframe scan in fixed frame

            //ros::Time last_icp_time_;

            sensor_msgs::Imu latest_imu_msg_;
            sensor_msgs::Imu last_used_imu_msg_;
            nav_msgs::Odometry latest_odom_msg_;
            nav_msgs::Odometry last_used_odom_msg_;

            std::vector<double> a_cos_;
            std::vector<double> a_sin_;

            sm_params input_;
            sm_result output_;
            LDP prev_ldp_scan_;

            kf_t current_kf_;
            // **** methods

            void processScan(LDP& curr_ldp_scan, const ros::Time& time);

            void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                    LDP& ldp);

            void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
            void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
            void imuCallback (const sensor_msgs::Imu::ConstPtr& imu_msg);
            void scanOdomCallBack(const sensor_msgs::LaserScanConstPtr &, const nav_msgs::OdometryConstPtr &);
            void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
            bool getBaseToLaserTf (const std::string& frame_id);

            void getPrediction(double& pr_ch_x, double& pr_ch_y,
                            double& pr_ch_a);

            void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);
    };
}

#endif