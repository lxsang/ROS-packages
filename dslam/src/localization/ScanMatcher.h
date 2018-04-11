#ifndef SCANMATCHER_H
#define SCANMATCHER_H

#include <pcl/registration/icp_nl.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/gicp.h>
#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include "3rd/line_extraction/line_extraction.h"
#include "3rd/laser_geometry/laser_geometry.h"
#include "utils/luaconf.h"

namespace dslam {
    typedef struct{
        Eigen::Quaterniond rot;
        Eigen::Vector4f tl;
        Eigen::Matrix4f tf;
        bool converged;
        double fitness;
    } icp_tf_t;
    typedef struct{
        pcl::PointCloud<pcl::PointXYZ> cloud;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d position;
    } feature_t;
    class LineScanMatcher{

        public:
            LineScanMatcher();
            ~LineScanMatcher(){};
            void setTF(tf::TransformListener* tf) {tf_ = tf;}
            void configure(Configuration&);
            void registerScan(const sensor_msgs::LaserScan::ConstPtr &, const nav_msgs::Odometry::ConstPtr&);
            void match(icp_tf_t&, const void (*)(std::vector<Line>&, pcl::PointCloud<pcl::PointXYZ>&));
            
        private:
            void cacheData(const sensor_msgs::LaserScan::ConstPtr&);
            void linesToPointCloud(std::vector<Line>& lines, pcl::PointCloud<pcl::PointXYZ>& cloud, tf::StampedTransform&);
            feature_t last_features_,current_feature_;
            void alignLastFeature(pcl::PointCloud<pcl::PointXYZ>&);
            //pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            //void getRotation(Eigen::Quaterniond &);
            pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
            //pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            
            LineExtraction line_extraction_;
            tf::TransformListener* tf_;
            int nscan_;
            double line_scale_;
            bool first_match_, tf_ok_;
            double sample_dist_, translation_tolerance_;
            std::string global_frame_, laser_frame_, robot_base_frame_;
            tf::StampedTransform laser2base_;
            double yaw;
    };

    class PointCloudScanMatcher{
        public:
            PointCloudScanMatcher();
            ~PointCloudScanMatcher();
            void configure(Configuration&);
            void registerScan(const sensor_msgs::LaserScan::ConstPtr &, geometry_msgs::Quaternion);
            void match(icp_tf_t&, const void (*callback)(pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>&));
            void setTF(tf::TransformListener* tf) {tf_ = tf;}
        private:
            feature_t last_features_, current_feature_;
            void rotateLastFeature(pcl::PointCloud<pcl::PointXYZ>& );
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            std::string global_frame_, laser_frame_, robot_base_frame_;
            double getYaw();
            bool first_match_;
            double sample_dist_, max_range_;
            laser_geometry::LaserProjection projector_;
            int nscan_;
            tf::TransformListener* tf_;
    };
}

#endif