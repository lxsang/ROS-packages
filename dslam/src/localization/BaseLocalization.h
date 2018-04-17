#ifndef BASE_LOCALIZATION_H
#define BASE_LOCALIZATION_H

#include <pcl/registration/registration.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/gicp.h>
#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include "3rd/line_extraction/line_extraction.h"
#include "utils/luaconf.h"
#include "common.h"

namespace dslam
{
    class BaseLocalization{
        public:
            BaseLocalization();
            virtual ~BaseLocalization();
            void setTF(tf::TransformListener* tf) {tf_ = tf;};
            virtual void configure(Configuration&);
            void registerScan(sensor_msgs::LaserScan &scan_msg, nav_msgs::Odometry& odom);
            virtual bool match(const void (*)(std::vector<Line>&, pcl::PointCloud<pcl::PointXYZ>&))= 0;
            void getTransform(tf::Transform&);
            void getLastKnownPose(geometry_msgs::Pose& pose);

            std::list<key_frame_t> keyframes;
        protected:
            void extractLines(std::vector<Line> &lines);
            void cacheData(sensor_msgs::LaserScan&);
            void linesToPointCloud(std::vector<Line>& lines, pcl::PointCloud<pcl::PointXYZ>& cloud, tf::StampedTransform&);
            landmark_t last_features_,current_feature_;
            //pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
            pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>* icp;
            LineExtraction line_extraction_;
            tf::TransformListener* tf_;
            int nscan_;
            double line_scale_, max_line_dist_;
            bool first_match_, tf_ok_;
            double sample_fitness_;
            std::string global_frame_, laser_frame_, robot_base_frame_;
            double keyframe_sample_linear_,keyframe_sample_angular_;
            tf::StampedTransform laser2base_;
            key_frame_t current_kf_;
            geometry_msgs::Pose pose_;
            int kf_idx_;
        private:
            pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp_nl_;
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_ln_;
            
    };
}

#endif