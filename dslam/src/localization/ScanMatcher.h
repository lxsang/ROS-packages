#ifndef SCANMATCHER_H
#define SCANMATCHER_H

#include <pcl/registration/icp.h>
#include <pcl/common/projection_matrix.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include "3rd/line_extraction/line_extraction.h"
#include "utils/luaconf.h"

namespace dslam {
    typedef struct{
        Eigen::Matrix4f tf;
        bool converged;
        double fitness;
    } icp_tf_t;
    class ScanMatcher{

        public:
            ScanMatcher(){nscan_ = 0;first_match_ = true;};
            ~ScanMatcher(){};
            void configure(Configuration&);
            void registerScan(const sensor_msgs::LaserScan::ConstPtr &);
            void match(icp_tf_t&, const void (*)(std::vector<Line>&, pcl::PointCloud<pcl::PointXYZ>&));
            
        private:
            void cacheData(const sensor_msgs::LaserScan::ConstPtr &);
            void linesToPointCloud(std::vector<Line>& lines, pcl::PointCloud<pcl::PointXYZ>& cloud);
            pcl::PointCloud<pcl::PointXYZ> last_features_;
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            LineExtraction line_extraction_;
            int nscan_;
            bool first_match_;
            double sample_dist_;
            std::string global_frame_, laser_frame_;
    };
}

#endif