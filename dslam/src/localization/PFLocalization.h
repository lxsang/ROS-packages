#ifndef PFLocalization_H
#define PFLocalization_H

// Particle filter
#include <filter/bootstrapfilter.h>
#include "NonLinearSystemPDF.h"
#include "NonLinearMeasurementPDF.h"

// ICP
#include <pcl/registration/icp_nl.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/gicp.h>


#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "3rd/line_extraction/line_extraction.h"
#include "3rd/laser_geometry/laser_geometry.h"
#include "utils/luaconf.h"
#include "common.h"
 // Default Number of Samples


namespace dslam {
    using namespace BFL;
    using namespace MatrixWrapper;

    class PFLocalization{
        public:
            PFLocalization();
            ~PFLocalization();
            void setTF(tf::TransformListener* tf) {tf_ = tf;};
            void configure(Configuration&);
            void registerScan(sensor_msgs::LaserScan &scan_msg, nav_msgs::Odometry& odom);
            bool match(const void (*)(std::vector<Line>&, pcl::PointCloud<pcl::PointXYZ>&));
            void getTransform(tf::Transform&);
            void getLastKnowPose(geometry_msgs::Pose& pose);

            std::list<key_frame_t> keyframes;
        private:
            void cacheData(sensor_msgs::LaserScan&);
            void linesToPointCloud(std::vector<Line>& lines, pcl::PointCloud<pcl::PointXYZ>& cloud, tf::StampedTransform&);
            feature_t last_features_,current_feature_;
            void alignLastFeature(pcl::PointCloud<pcl::PointXYZ>&);
            //void publishTranform(Eigen::Quaterniond& ori);
            //pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            //void getRotation(Eigen::Quaterniond &);
            pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
            //pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            //Eigen::Vector3d last_know_position_;
            LineExtraction line_extraction_;
            tf::TransformListener* tf_;
            int nscan_;
            double line_scale_;
            bool first_match_, tf_ok_;
            double sample_fitness_;
            std::string global_frame_, laser_frame_, robot_base_frame_;
            double keyframe_sample_linear_,keyframe_sample_angular_;
            tf::StampedTransform laser2base_;
            key_frame_t current_kf_;
            BootstrapFilter<ColumnVector,ColumnVector>* pf_filter_;
            SystemModel<ColumnVector>* sys_model_;
            MeasurementModel<ColumnVector,ColumnVector>* meas_model_;
            NonLinearSystemPDF* sys_pdf_;
            NonLinearMeasurementPDF* meas_pdf_;
            MCPdf<ColumnVector>* prior_discr_;
            //toRotationMatrix().eulerAngles(0, 1, 2)[2]
    };
}

#endif